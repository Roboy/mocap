#pragma once
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
// opencv
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// std
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <thread>
#include <future>
#define GLM_FORCE_RADIANS
#include <glm/detail/type_mat.hpp>
#include <glm/gtc/matrix_transform.hpp>
// ros
#include <ros/ros.h>
#include <communication/Vector2.h>
#include <communication/MarkerPosition.h>

#define MARKER 4
using namespace std;
using namespace Eigen;

typedef enum
{
    Uninitialized,
    Initialized,
    Tracking,
    Error
} CameraState;

typedef Matrix<double, 4, MARKER> Matrix4xMARKERd;
typedef Matrix<double, 3, MARKER> Matrix3xMARKERd;
typedef Matrix<double, 3, 4> Matrix3x4d;

// Generic functor for Eigen Levenberg-Marquardt minimizer
template<typename _Scalar, int NX=Dynamic, int NY=Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }
};

struct CameraMarkerModel:Functor<double>{
    CameraMarkerModel();
    int initializeModel(const communication::MarkerPosition::ConstPtr& msg);
    int track(const communication::MarkerPosition::ConstPtr& msg);

    void checkCorrespondence();
    void projectInto2D(Matrix3xMARKERd &position2d, Matrix4xMARKERd &position3d, Matrix3x4d &RT);
    void getRTmatrix(VectorXd &x, Matrix3x4d &RT);
    void getRTmatrix(Matrix4d &RT);
    void getRTmatrix(Matrix4f &RT);
    int operator()(const VectorXd &x, VectorXd &fvec) const;
    VectorXd pose;
    Matrix4d ModelMatrix;
    Matrix4d Trafo2FirstCamera;
    Matrix4xMARKERd pos3D;// ((x y z 1)' (x y z 1)' ... ),
    Vector4d origin3D;
    Vector3d origin2D;
    Matrix3xMARKERd pos2D;// ((x y 1)' (x y 1)' ... )
    cv::Mat img, img_rectified;
    cv::Mat map1, map2;
    cv::VideoCapture capture;
    uint id;
    char name[20];
    Matrix3d K;
    int threshold_value = 240;
    std::vector<uint> markerIDs;
    NumericalDiff<CameraMarkerModel> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CameraMarkerModel>, double> *lm;
    double reprojectionError;
};

class MarkerTracker{
public:
    /** Constructor */
    MarkerTracker();
    /** Destructor */
    ~MarkerTracker();
    /**
     * Sets initialized=false and waits for successful initialization of all cameras
     * @return success
     */
    bool init();
    /**
     * This is the callback function for the marker positions,
     * depending on the cameraID these a passed to the responsible Models
     * @param msg MarkerPosition message, containing seq, timestamp, position, cameraID
     */
    void pipe2function(const communication::MarkerPosition::ConstPtr& msg);

    Matrix4d ModelMatrix;
    Vector3d pos3D_old, pos3D_new;

    std::map<int, CameraMarkerModel> camera;
    std::map<int, int> cameraState;
private:
    ros::AsyncSpinner *spinner;
    ros::Subscriber marker_position_sub;
    bool initialized = false;
    ros::NodeHandle nh;
};

