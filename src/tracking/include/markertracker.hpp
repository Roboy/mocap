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
// ros
#include <ros/ros.h>
#include <communication/Vector2.h>
#include <communication/MarkerPosition.h>
#include <communication/CameraControl.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "cameraMarkerModel.hpp"

using namespace std;
using namespace Eigen;
using namespace cv;

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
     * depending on the cameraID and the cameras state, these are passed to the responsible Models
     * @param msg MarkerPosition message, containing seq, timestamp, position in image, cameraID
     */
    void pipe2function(const communication::MarkerPosition::ConstPtr& msg);

    bool sendCameraControl(uint ID, uint control, bool value);

    void videoCB(const sensor_msgs::ImageConstPtr& msg);

    Matrix4d ModelMatrix;

    ros::NodeHandle nh;
    ros::Subscriber marker_position_sub, video_sub;
    ros::Publisher rviz_marker_pub, camera_control_pub;
    map<int, CameraMarkerModel> camera;
    map<int, int> cameraState;
    vector<vector<float>> markerPositions;
    enum{
        toggleVideoStream = 0
    }cameraControls;
    cv_bridge::CvImageConstPtr cv_ptr;
    bool lockWhileWriting = false;
private:
    ros::AsyncSpinner *spinner;
    bool initialized = false;
};

