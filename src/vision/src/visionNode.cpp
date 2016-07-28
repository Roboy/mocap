#include "visionNode.hpp"

uint VisionNode::ID = 0;
int VisionNode::threshold_value = 240;
std::chrono::high_resolution_clock::time_point VisionNode::t1;
std::chrono::high_resolution_clock::time_point VisionNode::t2;
std::chrono::duration<double> VisionNode::time_span;
Mat VisionNode::img = Mat(HEIGHT, WIDTH, CV_8UC4, VisionNode::img_data);
Mat VisionNode::img_rectified = Mat(HEIGHT, WIDTH, CV_8UC4, VisionNode::img_rectified_data);
Mat VisionNode::img_gray = Mat(HEIGHT, WIDTH, CV_8UC1, VisionNode::img_gray_data);
ros::Publisher* VisionNode::marker_position_pub = NULL;
ros::Publisher* VisionNode::video_pub = NULL;
unsigned char* VisionNode::img_data = new unsigned char[WIDTH*HEIGHT*4];
unsigned char* VisionNode::img_rectified_data = new unsigned char[WIDTH*HEIGHT*4];
unsigned char* VisionNode::img_gray_data = new unsigned char[WIDTH*HEIGHT];
Mat VisionNode::map1;
Mat VisionNode::map2;
bool VisionNode::publish_video_flag = false;

VisionNode::VisionNode() {
    cv::FileStorage fs("/home/letrend/workspace/mocap/src/intrinsics.xml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("could not open intrinsics.xml");
        return;
    }
    fs["camera_matrix"] >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    ID = 0;

    // calculate undistortion mapping
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
                            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(WIDTH, HEIGHT), 1,
                                                      cv::Size(WIDTH, HEIGHT), 0),
                            cv::Size(WIDTH, HEIGHT), CV_16SC2, map1, map2);

    marker_position_pub = new ros::Publisher;
    *marker_position_pub = nh.advertise<communication::MarkerPosition>("/raspicamera/marker_position", 1000);

    video_pub = new ros::Publisher;
    *video_pub = nh.advertise<sensor_msgs::Image>("/raspicamera/video", 1);

    camera_control_sub = nh.subscribe("/camera_control", 1000, &VisionNode::camera_control, this);

    // Publish the marker
    while (marker_position_pub->getNumSubscribers() < 1) {
        ros::Duration d(1.0);
        if (!ros::ok()) {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker position");
        d.sleep();
    }
    ROS_INFO_ONCE("Found subscriber");

    spinner = new ros::AsyncSpinner(1);
    spinner->start();

    img = cv::Mat(HEIGHT, WIDTH, CV_8UC4, img_data);
    img_rectified = cv::Mat(HEIGHT, WIDTH, CV_8UC4, img_rectified_data);

    t1 = std::chrono::high_resolution_clock::now();

    StartCamera(WIDTH, HEIGHT, 90, CameraCallback);
}

VisionNode::~VisionNode() {
    spinner->stop();
    delete spinner;
    delete marker_position_pub;
    delete video_pub;
    delete[] img_data;
    delete[] img_rectified_data;
    delete[] img_gray_data;
}

void VisionNode::camera_control(const communication::CameraControl::ConstPtr& msg){
    if(msg->cameraID == ID){ // only react to control for this camera
        switch(msg->control) {
            case toggleVideoStream:
                publish_video_flag = msg->value1;
                break;
        }
    }
}

void VisionNode::CameraCallback(CCamera *cam, const void *buffer, int buffer_length) {
    cv::Mat myuv(HEIGHT + HEIGHT / 2, WIDTH, CV_8UC1, (unsigned char *) buffer);
    cv::cvtColor(myuv, img, CV_YUV2RGBA_NV21);
    cv::cvtColor(img, img_gray, CV_RGBA2GRAY);

    communication::MarkerPosition markerPosition;
    markerPosition.header.stamp = ros::Time::now();
    static uint next_id = 0;
    markerPosition.header.seq = next_id++;
    markerPosition.cameraID = ID;

    t2 = std::chrono::high_resolution_clock::now();
    time_span = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
    markerPosition.fps = (double)next_id/time_span.count();

    cv::Mat filtered_img;
    cv::threshold(img_gray, filtered_img, threshold_value, 255, 3);

    // find contours in result, which hopefully correspond to a found object
    vector <vector<cv::Point>> contours;
    vector <cv::Vec4i> hierarchy;
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
    vector <cv::Point2f> centers(contours.size());
    vector<float> radius(contours.size());
    for (int idx = 0; idx < contours.size(); idx++) {
        //drawContours(img, contours, idx, cv::Scalar(255, 0, 0), 4, 8, hierarchy, 0,
        //             cv::Point());
        minEnclosingCircle(contours[idx], centers[idx], radius[idx]);
        communication::Vector2 pos;
        pos.x = centers[idx].x;
        pos.y = centers[idx].y;
        markerPosition.marker_position.push_back(pos);
    }
    //imshow("camera", img);
    //waitKey(1);
    marker_position_pub->publish(markerPosition);

    if(publish_video_flag){
        cv_bridge::CvImage cvImage;
        img_gray.copyTo(cvImage.image);
        cvImage.encoding = "mono8";
        sensor_msgs::Image msg;
        cvImage.toImageMsg(msg);
	msg.encoding = "mono8";
	msg.header = markerPosition.header; 
        video_pub->publish(msg);
    }
}
