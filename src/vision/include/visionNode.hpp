#pragma once

// ros
#include <ros/ros.h>
#include <communication/MarkerPosition.h>

// opencv
#include "opencv2/opencv.hpp"

// picam
#include "camera.h"
#include <opencv2/opencv.hpp>

#define WIDTH 640
#define HEIGHT 480

using namespace std;
using namespace cv;

class VisionNode{
public:
    VisionNode();
    ~VisionNode();

private:
    void CameraCallback(CCamera* cam, const void* buffer, int buffer_length);
    int threshold_value = 240;
    unsigned char dest[WIDTH*HEIGHT*4];
    cv::Mat img;
    communication::MarkerPosition markerPosition;
    Mat cameraMatrix, distCoeffs;
    Mat img_rectified, map1, map2;
    Mat myuv, img;
    ros::NodeHandle nh;
    ros::Publisher marker_position_pub;
    ros::Subscriber initialize_sub;
    ros::AsyncSpinner *spinner;
    uint next_id = 0;
};