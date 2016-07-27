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

Mat img, img_rectified;
unsigned char img_data[WIDTH*HEIGHT*4], img_rectified_data[WIDTH*HEIGHT*4];

static std::chrono::high_resolution_clock::time_point t1;
static std::chrono::high_resolution_clock::time_point t2;
static std::chrono::duration<double> time_span;

class VisionNode{
public:
    VisionNode();
    ~VisionNode();

private:
    static void CameraCallback(CCamera* cam, const void* buffer, int buffer_length);
    int threshold_value = 240;
    uint ID;
    Mat cameraMatrix, distCoeffs;
    Mat img_rectified, map1, map2;
    Mat myuv;
    ros::NodeHandle nh;
    ros::Publisher marker_position_pub;
    ros::Subscriber initialize_sub;
    ros::AsyncSpinner *spinner;
};
