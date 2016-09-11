// std
#include <string>
#include <vector>
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
// messages
#include <communication/MarkerPosition.h>
// ros
#include <ros/ros.h>

#define MARKER 4
using namespace std;
using namespace Eigen;
using namespace cv;

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
    /**
     * This function initializes the markerModel
     * @param msg Message from Camera containing 2D marker positions in image
     * @return Uninitialized if reprojectionError < 2.0, else Initialized
     */
    int initializeModel(const communication::MarkerPosition::ConstPtr& msg);
    /**
     * This function tracks the markerModel
     * @param msg Message from Camera containing 2D marker positions in image
     * @return Tracking if 4 Marker are visible, else Error
     */
    int track(const communication::MarkerPosition::ConstPtr& msg);
    /**
     * This function permutes every possible combination of 3D MarkerPositions and sets the one
     * with the lowest reprojectionError
     */
    void checkCorrespondence();
    /**
     * This function projects the 3D markerPositions of the tracking result into 2D image plane positions
     */
    void updateProjectedMarkerPositions();
    /**
     * projects 3D points into 2D using the camera intrinsics matrix K
     * @param position2d 2D projected positions
     * @param position3d 3D positions
     * @param RT pose matrix
     */
    void projectInto2D(Matrix3xMARKERd &position2d, Matrix4xMARKERd &position3d, Matrix3x4d &RT);
    /**
     * This function calculates the pose matrix from the current pose of the tracked model
     * @param RT reference to double pose matrix (will be filled)
     */
    template<typename T> void getRTmatrix(T &RT);
    /**
     * Function used by minimizer
     * @param x pose vector (3 translation, 3 rotation parameters)
     * @param fvec reprojection error
     * @return minimization state
     */
    int operator()(const VectorXd &x, VectorXd &fvec) const;
    VectorXd pose;
    Matrix4d ModelMatrix;
    Matrix4d Trafo2FirstCamera;
    Matrix4xMARKERd pos3D;// ((x y z 1)' (x y z 1)' ... ),
    Vector4d origin3D;
    Vector3d origin2D;
    Matrix3xMARKERd pos2D;// ((x y 1)' (x y 1)' ... )
    Matrix3xMARKERd projectedPosition2D;// ((x y 1)' (x y 1)' ... )
    cv::Mat img, img_rectified;
    cv::Mat map1, map2;
    cv::VideoCapture capture;
    int id;
    char name[20];
    Matrix3d K;
    int threshold_value = 240;
    std::vector<uint> markerIDs;
    NumericalDiff<CameraMarkerModel> *numDiff;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CameraMarkerModel>, double> *lm;
    double reprojectionError;
    uint markerVisible = 0;
    float fps = 0;
};