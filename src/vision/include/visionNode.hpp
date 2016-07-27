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

Mat img;

class VisionNode{
public:
    VisionNode();
    ~VisionNode();

private:
    static void CameraCallback(CCamera* cam, const void* buffer, int buffer_length);
    int threshold_value = 240;
    unsigned char dest[WIDTH*HEIGHT*4];
    Mat cameraMatrix, distCoeffs;
    Mat img_rectified, map1, map2;
    Mat myuv;
    ros::NodeHandle nh;
    ros::Publisher marker_position_pub;
    ros::Subscriber initialize_sub;
    ros::AsyncSpinner *spinner;
};
