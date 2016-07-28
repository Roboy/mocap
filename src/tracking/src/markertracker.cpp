#include "markertracker.hpp"

CameraMarkerModel::CameraMarkerModel(void): Functor<double>(6,2*MARKER){
    pose = VectorXd(6);
    markerIDs.resize(MARKER);
    Trafo2FirstCamera = Matrix4d::Identity();

    cv::Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("/home/letrend/workspace/markertracker/intrinsics.xml",cv::FileStorage::READ);
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
    // this is the representation of the marker
    pos3D(0,0)=0;       pos3D(1,0)=-0.123;   pos3D(2,0)=0;       pos3D(3,0)=1;
    pos3D(0,1)=0;       pos3D(1,1)=0;       pos3D(2,1)=-0.08;   pos3D(3,1)=1;
    pos3D(0,2)=-0.105;   pos3D(1,2)=0;       pos3D(2,2)=0;       pos3D(3,2)=1;
    pos3D(0,3)=0.182;  pos3D(1,3)=0;       pos3D(2,3)=0;  pos3D(3,3)=1;

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
        pos2D(0, 0) = positions[1].y > positions[2].y ? positions[2].x : positions[1].x;
        pos2D(1, 0) = positions[1].y > positions[2].y ? positions[2].y : positions[1].y;
        pos2D(2, 0) = 1;
        pos2D(0, 1) = positions[1].y < positions[2].y ? positions[2].x : positions[1].x;
        pos2D(1, 1) = positions[1].y < positions[2].y ? positions[2].y : positions[1].y;
        pos2D(2, 1) = 1;
        pos2D(0, 2) = positions[0].x;
        pos2D(1, 2) = positions[0].y;
        pos2D(2, 2) = 1;
        pos2D(0, 3) = positions[3].x;
        pos2D(1, 3) = positions[3].y;
        pos2D(2, 3) = 1;
//    cout << "pos2D: \n" << pos2D << endl;

        for (uint id = 0; id < MARKER; id++)
            markerIDs[id] = id;

        // find the pose
        pose << 0, 0, 0, 0, 0, 1;

        numDiff = new NumericalDiff<CameraMarkerModel>(*this);
        lm = new LevenbergMarquardt<NumericalDiff<CameraMarkerModel>, double>(*numDiff);
        lm->parameters.maxfev = 2000;
        lm->parameters.xtol = 1.0e-10;
        int ret = lm->minimize(pose);
//    cout << "iterations: " << lm->iter << endl;
//    cout << "x that minimizes the function: \n" << pose << endl;

        Matrix3x4d RT;
        getRTmatrix(pose, RT);

        Matrix3xMARKERd projectedPosition2D = K * RT * pos3D;
        origin2D = K * RT * origin3D;
        origin2D(0) /= origin2D(2);
        origin2D(1) /= origin2D(2);
        char str[1];
        for (uint col = 0; col < MARKER; col++) {
            projectedPosition2D(0, col) /= projectedPosition2D(2, col);
            projectedPosition2D(1, col) /= projectedPosition2D(2, col);

            cv::Point2f center(projectedPosition2D(0, col), projectedPosition2D(1, col));
            circle(img, center, 5, cv::Scalar(255, 255, 0), 4);
            line(img, cv::Point2f(origin2D(0), origin2D(1)), center, cv::Scalar::all(255), 4);
            sprintf(str, "%d", markerIDs[col]);
            putText(img, str, center, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar::all(255));
        }
//    cout << "projectedPosition2D: \n" << projectedPosition2D << endl;

        cv::imshow(name, img);
        cv::waitKey(1);

        delete numDiff;
        delete lm;

        return CameraState::Initialized;
    }else{
        return CameraState::Uninitialized;
    }
}

int CameraMarkerModel::track(const communication::MarkerPosition::ConstPtr& msg){
    reprojectionError = 1000000;

    if (msg->marker_position.size() == 4) {
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
        lm->parameters.maxfev = 500;
        lm->parameters.xtol = 1.0e-10;
        int ret = lm->minimize(pose);
//                    cout << "iterations: " << lm->iter << endl;
//                    cout << "x that minimizes the function: \n" << pose << endl;

        Matrix4d RT;
        getRTmatrix(RT);
//                    cout << "RT: \n" << RT << endl;
        pos3D = RT*pos3D;
        ModelMatrix = RT*ModelMatrix;
//            cout << "ModelMatrix : \n" << ModelMatrix  << endl;

        Matrix3xMARKERd projectedPosition2D = K * pos3D.block<3,MARKER>(0,0);
        origin2D = K * ModelMatrix.topRightCorner(3,1);
        origin2D(0)/=origin2D(2);
        origin2D(1)/=origin2D(2);
        char name[1];
        for(uint col = 0; col<MARKER; col ++){
            projectedPosition2D(0,col)/=projectedPosition2D(2,col);
            projectedPosition2D(1,col)/=projectedPosition2D(2,col);

            float radius=10;
            cv::Point2f center(projectedPosition2D(0,col), projectedPosition2D(1,col));
            circle(img, center, radius, cv::Scalar(255, 0, 0), 4);
            line(img, cv::Point2f(origin2D(0),origin2D(1)), center, cv::Scalar::all(255),4);
            sprintf(name,"%d",markerIDs[col]);
            putText(img,name,center,cv::FONT_HERSHEY_SCRIPT_SIMPLEX,1,cv::Scalar::all(0));
        }

        reprojectionError = lm->fnorm;

        delete numDiff;
        delete lm;
        return CameraState::Tracking;
    }else{
        return CameraState::Error;
    }
}

void CameraMarkerModel::checkCorrespondence(){
    Matrix3x4d RT;
    getRTmatrix(pose,RT);
    Matrix4xMARKERd pos3D_backup = pos3D;

    vector<uint> perm = {0,1,2,3};
    double minError = 1e10;
    vector<uint> bestPerm = {0,1,2,3};
    cv::Mat img = cv::Mat::zeros(cv::Size(640,480),CV_8UC3);

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

void CameraMarkerModel::projectInto2D(Matrix3xMARKERd &position2d, Matrix4xMARKERd &position3d, Matrix3x4d &RT) {
    position2d = K * RT * position3d;
    for(uint col = 0; col<MARKER; col ++){
        position2d(0,col)/=position2d(2,col);
        position2d(1,col)/=position2d(2,col);
    }
}

void CameraMarkerModel::getRTmatrix(VectorXd &x, Matrix3x4d &RT){
    RT = MatrixXd::Identity(3,4);
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

void CameraMarkerModel::getRTmatrix(Matrix4d &RT){
    RT = Matrix4d::Identity();
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

void CameraMarkerModel::getRTmatrix(Matrix4f &RT){
    RT = Matrix4f::Identity();
    // construct quaternion (cf unit-sphere projection Terzakis paper)
    float alpha_squared = powf(powf(pose(0),2.0f)+powf(pose(1),2.0f)+powf(pose(2),2.0f),2.0f);
    Quaternionf q((1-alpha_squared)/(alpha_squared+1),
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

MarkerTracker::MarkerTracker(){
    spinner = new ros::AsyncSpinner(5);
    spinner->start();
    marker_position_sub = nh.subscribe("/raspicamera/marker_position", 1000, &MarkerTracker::pipe2function, this);
    rviz_marker_pub=nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);
    camera_control_pub=nh.advertise<communication::CameraControl>("/camera_control", 1000);
    video_sub=nh.subscribe("/raspicamera/video", 1, &MarkerTracker::videoCB, this);
}

MarkerTracker::~MarkerTracker() {
    spinner->stop();
    delete spinner;
}

bool MarkerTracker::init() {
    ROS_INFO("please stand in front of cameras");
    initialized = false;
    ros::Duration d(1.0);
    while(!initialized)
        d.sleep();
    return true;
}

void MarkerTracker::pipe2function(const communication::MarkerPosition::ConstPtr& msg){
    // check if camera was already registered
    auto it = camera.find(msg->cameraID);
    if(it != camera.end()) // camera already registered
    {
        camera[msg->cameraID].markerVisible = msg->markerVisible;
        camera[msg->cameraID].fps = msg->fps;
        switch(cameraState[msg->cameraID]){
            case Uninitialized:{
                cameraState[msg->cameraID] = it->second.initializeModel(msg);
                break;
            }
            case Initialized:{
                bool allInitialized = true;
                for(auto state = cameraState.begin(); state != cameraState.end(); ++state){
                    if(state->second==Uninitialized)
                        allInitialized = false;
                }
                if(allInitialized) { // if all cameras are initialized, set initialized flag and cameraState to Tracking
                    initialized = true;
                    for(auto state = cameraState.begin(); state != cameraState.end(); ++state) {
                        state->second = Tracking; // TODO: possibly find trafo between cameras here
                    }
                }else{
                    initialized = false;
                }
                break;
            }
            case Tracking:{
                cameraState[msg->cameraID] = it->second.track(msg);
                break;
            }
            case Error:{
                cameraState[msg->cameraID] = Tracking; // TODO: error handling
                break;
            }
        }
    }else{ // register new camera
        camera[msg->cameraID].id = msg->cameraID;
        sprintf(camera[msg->cameraID].name, "camera %d", msg->cameraID);
        cameraState[msg->cameraID] = Uninitialized;
    }
}

bool MarkerTracker::sendCameraControl(uint ID, uint control, bool value){
    // check if camera exists
    auto it = camera.find(ID);
    if(it != camera.end())
    {
        communication::CameraControl msg;
        msg.cameraID = ID;
        msg.control = control;
        switch(control) {
            case toggleVideoStream:
                msg.value1 = value;
                break;
            default:
                return false;
        }
        camera_control_pub.publish(msg);
    }
    return false;
}

void MarkerTracker::videoCB(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // Update GUI Window
    cv::imshow("video stream", cv_ptr->image);
    cv::waitKey(1);
}