#include "cameraMarkerModel.hpp"

CameraMarkerModel::CameraMarkerModel(void): Functor<double>(6,2*MARKER){
    pose = VectorXd(6);
    markerIDs.resize(MARKER);
    Trafo2FirstCamera = Matrix4d::Identity();

    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("/home/roboy/workspace/mocap/src/intrinsics.xml",cv::FileStorage::READ);
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    // calculate undistortion mapping
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(640,480), 1, cv::Size(640,480), 0),
                                cv::Size(640,480), CV_16SC2, map1, map2);

    //std::cout<< "cameraMatrix: \n" << cameraMatrix << "\ndistCoeffs: \n" << distCoeffs << std::endl;

    // camera intrinsic matrix
    K = Matrix3d((double*)cameraMatrix.data).transpose();
    //cout << "camera instrinsics:\n" << K << endl;
};

int CameraMarkerModel::initializeModel(const communication::MarkerPosition::ConstPtr& msg){
    // this is the representation of the marker, when initializing, this is the first estimate
    pos3D(0,0)=0.133;  pos3D(1,0)=0;       pos3D(2,0)=0;  pos3D(3,0)=1;
    pos3D(0,1)=0;       pos3D(1,1)=0;       pos3D(2,1)=0.05;   pos3D(3,1)=1;
    pos3D(0,2)=-0.073;   pos3D(1,2)=0;       pos3D(2,2)=0;       pos3D(3,2)=1;
    pos3D(0,3)=0;       pos3D(1,3)=-0.104;   pos3D(2,3)=0;       pos3D(3,3)=1;

    origin3D << 0,0,0,1;
    origin2D << 0,0,1;

    ModelMatrix = Matrix4d::Identity();

    if (msg->marker_position.size() == MARKER) {
        // sort the centers wrt x coordinates
        vector<communication::Vector2> positions = msg->marker_position;
        sort(positions.begin(), positions.end(),
             [](const communication::Vector2 &a, const communication::Vector2 &b) -> bool {
                 return a.x <= b.x;
             });

        // write those positions to the model
        pos2D(0, 0) = positions[0].x;
        pos2D(1, 0) = positions[0].y;
        pos2D(2, 0) = 1;
        markerIDs[0] = 0;
        pos2D(0, 1) = positions[1].y > positions[2].y ? positions[1].x : positions[2].x;
        pos2D(1, 1) = positions[1].y > positions[2].y ? positions[1].y : positions[2].y;
        pos2D(2, 1) = 1;
        markerIDs[1] = 1;
        pos2D(0, 2) = positions[3].x;
        pos2D(1, 2) = positions[3].y;
        pos2D(2, 2) = 2;
        markerIDs[2] = 2;
        pos2D(0, 3) = positions[1].y < positions[2].y ? positions[1].x : positions[2].x;
        pos2D(1, 3) = positions[1].y < positions[2].y ? positions[1].y : positions[2].y;
        pos2D(2, 3) = 1;
        markerIDs[3] = 3;

//    cout << "pos2D: \n" << pos2D << endl;


        // find the pose
        pose << 0, 0, 0, 0, 0, 1;

        numDiff = new NumericalDiff<CameraMarkerModel>(*this);
        lm = new LevenbergMarquardt<NumericalDiff<CameraMarkerModel>, double>(*numDiff);
        lm->parameters.maxfev = 2000;
        lm->parameters.xtol = 1.0e-10;
        int ret = lm->minimize(pose);
//    cout << "iterations: " << lm->iter << endl;
//    cout << "x that minimizes the function: \n" << pose << endl;

        reprojectionError = lm->fnorm;

        updateProjectedMarkerPositions();

        delete numDiff;
        delete lm;

        if(reprojectionError < 2.0)
            return CameraState::Initialized;
        else
            return CameraState::Uninitialized;
    }else{
        return CameraState::Uninitialized;
    }
}

int CameraMarkerModel::track(const communication::MarkerPosition::ConstPtr& msg){
    reprojectionError = 1000000;

    if (msg->marker_position.size() == MARKER) {
        // copy points
        for (int idx = 0; idx < msg->marker_position.size(); idx++) {
            pos2D(0, idx) = msg->marker_position[idx].x;
            pos2D(1, idx) = msg->marker_position[idx].y;
            pos2D(2, idx) = 1;
        }
        checkCorrespondence();

        // need to instantiate new lm every iteration so it will be using the new positions
        numDiff = new NumericalDiff<CameraMarkerModel>(*this);
        lm = new Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CameraMarkerModel>, double> (*numDiff);
        lm->parameters.maxfev = 2000;
        lm->parameters.xtol = 1.0e-10;
        int ret = lm->minimize(pose);
//                    cout << "iterations: " << lm->iter << endl;
//                    cout << "x that minimizes the function: \n" << pose << endl;

        reprojectionError = lm->fnorm;

        updateProjectedMarkerPositions();

        delete numDiff;
        delete lm;
        return CameraState::Tracking;
    }else{
        return CameraState::Error;
    }
}

void CameraMarkerModel::checkCorrespondence(){
    Matrix3x4d RT;
    getRTmatrix<Matrix3x4d>(RT);
    Matrix4xMARKERd pos3D_backup = pos3D;

    vector<uint> perm = {0,1,2,3};
    double minError = 1e10;
    vector<uint> bestPerm = {0,1,2,3};

    // permute all marker assignments and take the one with lowest reprojection error
    Matrix3xMARKERd projectedPosition2D(3,MARKER);
    do {
        pos3D <<
              pos3D_backup(0,perm[0]), pos3D_backup(0,perm[1]), pos3D_backup(0,perm[2]), pos3D_backup(0,perm[3]),
                pos3D_backup(1,perm[0]), pos3D_backup(1,perm[1]), pos3D_backup(1,perm[2]), pos3D_backup(1,perm[3]),
                pos3D_backup(2,perm[0]), pos3D_backup(2,perm[1]), pos3D_backup(2,perm[2]), pos3D_backup(2,perm[3]),
                1,1,1,1;
        projectInto2D(projectedPosition2D, pos3D, RT);
        Matrix<double,2,MARKER> difference;
        difference = projectedPosition2D.block<2,MARKER>(0,0)-pos2D.block<2,MARKER>(0,0);
        double error = difference.squaredNorm();

        if (error < minError) {
            bestPerm = perm;
            minError = error;
        }
    } while (next_permutation(perm.begin(), perm.end()));

    vector<uint> markerIDs_temp = markerIDs;
    for(uint id=0; id<MARKER; id++)
        markerIDs[id] = markerIDs_temp[bestPerm[id]];
//    printf("assignement: %d %d %d %d, error: %f\n", bestPerm[0], bestPerm[1], bestPerm[2], bestPerm[3], minError);

    pos3D <<
          pos3D_backup(0, bestPerm[0]), pos3D_backup(0, bestPerm[1]), pos3D_backup(0, bestPerm[2]), pos3D_backup(0, bestPerm[3]),
            pos3D_backup(1, bestPerm[0]), pos3D_backup(1, bestPerm[1]), pos3D_backup(1, bestPerm[2]), pos3D_backup(1, bestPerm[3]),
            pos3D_backup(2, bestPerm[0]), pos3D_backup(2, bestPerm[1]), pos3D_backup(2, bestPerm[2]), pos3D_backup(2, bestPerm[3]),
            1, 1, 1, 1;
}

void CameraMarkerModel::updateProjectedMarkerPositions(){
    Matrix4d RT;
    getRTmatrix<Matrix4d>(RT);
//                    cout << "RT: \n" << RT << endl;
    pos3D = RT*pos3D;
    ModelMatrix = RT*ModelMatrix;
//            cout << "ModelMatrix : \n" << ModelMatrix  << endl;

    projectedPosition2D = K * pos3D.block<3,MARKER>(0,0);
    origin2D = K * ModelMatrix.topRightCorner(3,1);
    origin2D(0)/=origin2D(2);
    origin2D(1)/=origin2D(2);
    for(uint col = 0; col<MARKER; col ++){
        projectedPosition2D(0,col)/=projectedPosition2D(2,col);
        projectedPosition2D(1,col)/=projectedPosition2D(2,col);
    }
}

void CameraMarkerModel::projectInto2D(Matrix3xMARKERd &position2d, Matrix4xMARKERd &position3d, Matrix3x4d &RT) {
    position2d = K * RT * position3d;
    for(uint col = 0; col<MARKER; col ++){
        position2d(0,col)/=position2d(2,col);
        position2d(1,col)/=position2d(2,col);
    }
}

template<typename T> void CameraMarkerModel::getRTmatrix(T &RT){
    RT = T::Identity();
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(pose(0),2.0)+pow(pose(1),2.0)+pow(pose(2),2.0),2.0);
    Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                  2.0*pose(0)/(alpha_squared+1),
                  2.0*pose(1)/(alpha_squared+1),
                  2.0*pose(2)/(alpha_squared+1));
    // construct RT matrix
    RT.topLeftCorner(3,3) = q.toRotationMatrix();
    RT.topRightCorner(3,1) << pose(3), pose(4), pose(5);
}

int CameraMarkerModel::operator()(const VectorXd &x, VectorXd &fvec) const
{
    Matrix3xMARKERd projectedPosition2D(3,MARKER);
    Matrix3x4d RT = MatrixXd::Identity(3,4);
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    double alpha_squared = pow(pow(x(0),2.0)+pow(x(1),2.0)+pow(x(2),2.0),2.0);
    Quaterniond q((1-alpha_squared)/(alpha_squared+1),
                  2.0*x(0)/(alpha_squared+1),
                  2.0*x(1)/(alpha_squared+1),
                  2.0*x(2)/(alpha_squared+1));
    // construct RT matrix
    RT.topLeftCorner(3,3) = q.toRotationMatrix();
    RT.topRightCorner(3,1) << x(3), x(4), x(5);

    projectedPosition2D = K * RT * pos3D;
    for(uint col = 0; col<MARKER; col ++){
        projectedPosition2D(0,col)/=projectedPosition2D(2,col);
        projectedPosition2D(1,col)/=projectedPosition2D(2,col);
    }

    Matrix<double,2,MARKER> difference;
    difference = projectedPosition2D.block<2,MARKER>(0,0)-pos2D.block<2,MARKER>(0,0);
    fvec << difference(0,0),difference(1,0),difference(0,1),difference(1,1),difference(0,2),difference(1,2),difference(0,3),difference(1,3);
//        cout << "K\n" << K << endl;
//        cout << "RT\n" << RT << endl;
//        cout << "position3D\n" <<  pos3D << endl;
//        cout << "position2D\n" <<  pos2D << endl;
//        cout << "projectedPosition2D\n" <<  projectedPosition2D << endl;
//        cout << "difference : " << difference <<endl;
//        cout << "error : " << difference.squaredNorm() <<endl;
//        cout << "x : " << x <<endl;
    return 0;
}