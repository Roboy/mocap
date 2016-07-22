// ros
#include <ros/ros.h>
#include <communication/MarkerPosition.h>

// opencv
#include "opencv2/opencv.hpp"

// raspicamera
#include <raspicam/raspicam_cv.h>

#define WIDTH 2592
#define HEIGHT 1944

int threshold_value = 240;

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    Mat cameraMatrix, distCoeffs;
    cv::FileStorage fs("/home/letrend/workspace/mocap/src/intrinsics.xml", cv::FileStorage::READ);
    if(!fs.isOpened()){
        ROS_ERROR("could not open intrinsics.xml");
        return -1;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    // calculate undistortion mapping
    Mat img_rectified, map1, map2;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(WIDTH, HEIGHT), 1,
                                                      cv::Size(WIDTH, HEIGHT), 0),
                            cv::Size(WIDTH, HEIGHT), CV_16SC2, map1, map2);

    ros::init(argc, argv, "visionNode");
    ros::NodeHandle nh;
    ros::Publisher marker_position_pub;
    ros::Subscriber initialize_sub;
    marker_position_pub = nh.advertise<communication::MarkerPosition>("/raspicamera/marker_position", 1000);

    // Publish the marker
    while (marker_position_pub.getNumSubscribers() < 1)
    {
        ros::Duration d(1.0);
        if (!ros::ok())
        {
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker position");
        d.sleep();
    }
    ROS_INFO_ONCE("Found subscriber");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    raspicam::RaspiCam_Cv camera;

    //set camera params
    camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
    //Open camera
    ROS_INFO("Opening Camera...");
    if (!camera.open()) {
        ROS_ERROR("Error opening the camera");
        return -1;
    }

    while (ros::ok()){
        static uint next_id = 0;
        communication::MarkerPosition markerPosition;
        markerPosition.header.frame_id = "map";
        markerPosition.header.stamp = ros::Time::now();
        markerPosition.header.seq = next_id++;

	markerPosition.cameraID = 0;

        cv::Mat img;
        camera.grab();
        camera.retrieve ( img);

        if (img.empty())
            continue;
        // undistort
        cv::remap(img, img_rectified, map1, map2, cv::INTER_CUBIC);
        cv::flip(img_rectified, img_rectified, 1);
        cv::Mat img_gray;

        cv::Mat filtered_img;
        cv::threshold(img_rectified, filtered_img, threshold_value, 255, 3);

        cv::Mat erodeElement = cv::getStructuringElement(2, cv::Size(3, 3), cv::Point(1, 1));
        cv::Mat dilateElement = cv::getStructuringElement(2, cv::Size(5, 5), cv::Point(3, 3));

        erode(filtered_img, filtered_img, erodeElement);
        erode(filtered_img, filtered_img, erodeElement);
        dilate(filtered_img, filtered_img, dilateElement);
        dilate(filtered_img, filtered_img, dilateElement);

        // find contours in result, which hopefully correspond to a found object
        vector<vector<cv::Point> > contours;
        vector<cv::Vec4i> hierarchy;
        findContours(filtered_img, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE,
                     cv::Point(0, 0));

        // filter out tiny useless contours
        double min_contour_area = 60;
        for (auto it = contours.begin(); it != contours.end();) {
            if (contourArea(*it) < min_contour_area) {
                it = contours.erase(it);
            }
            else {
                ++it;
            }
        }

        // get centers and publish
        vector<cv::Point2f> centers(contours.size());
        vector<float> radius(contours.size());
        for (int idx = 0; idx < contours.size(); idx++) {
            drawContours(img, contours, idx, cv::Scalar(255, 0, 0), 4, 8, hierarchy, 0,
                         cv::Point());
            minEnclosingCircle(contours[idx], centers[idx], radius[idx]);
            communication::Vector2 pos;
            pos.x = centers[idx].x;
            pos.y = centers[idx].y;
            markerPosition.marker_position.push_back(pos);
        }
        marker_position_pub.publish(markerPosition);

	//cv::imshow("raspi camera", img);
	//cv::waitKey(0);
    }

    camera.release();

    return 0;
}
