#pragma once

// ros
#include <ros/ros.h>
#include <communication/MarkerPosition.h>
#include <communication/CameraControl.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// opencv
#include "opencv2/opencv.hpp"

// picam
#include "camera.h"
#include <opencv2/opencv.hpp>

//std 
#include <chrono>

#define WIDTH 640
#define HEIGHT 480

using namespace std;
using namespace cv;


class VisionNode{
public:
    VisionNode();
    ~VisionNode();

private:
    void camera_control(const communication::CameraControl::ConstPtr& msg);
    static void CameraCallback(CCamera* cam, const void* buffer, int buffer_length);
    static int threshold_value;
    static std::chrono::high_resolution_clock::time_point t1;
    static std::chrono::high_resolution_clock::time_point t2;
    static std::chrono::duration<double> time_span;
    static uint ID;
    Mat cameraMatrix, distCoeffs;
    static Mat map1, map2;
    Mat myuv;
    ros::NodeHandle nh;
    static ros::Publisher *marker_position_pub, *video_pub;
    ros::Subscriber camera_control_sub;
    ros::AsyncSpinner *spinner;
    static Mat img, img_rectified, img_gray;
    static unsigned char *img_data, *img_rectified_data, *img_gray_data;
    static bool publish_video_flag;
    enum{
        toggleVideoStream = 0
    }cameraControls;
};
