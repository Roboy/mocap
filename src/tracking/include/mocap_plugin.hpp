#pragma once

#ifndef Q_MOC_RUN
// ros
#include <ros/ros.h>
#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//std
#include <stdio.h>
#include <map>
// qt
#include <QPainter>
#include <QCheckBox>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTableWidget>
#include <QComboBox>
#include <QTimer>
#include <QSlider>
// messages
#include "communication/CameraControl.h"
#include "communication/MarkerPosition.h"
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include "cameraMarkerModel.hpp"
// common definitions
#include "CommonDefinitions.hpp"

#endif

using namespace std;
using namespace cv;

class LightWidget : public QWidget {
Q_OBJECT
public:
    LightWidget(QWidget *parent = 0)
            : QWidget(parent){
        this->setFixedWidth(20);
        this->setFixedHeight(20);
    }

    void setColor(QColor color) {
        m_color = color;
    }

protected:
    virtual void paintEvent(QPaintEvent *){
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.setBrush(m_color);
        painter.drawEllipse(0, 0, width(), height());
    }

private:
    QColor m_color;
};

class MocapPlugin : public rviz::Panel {
Q_OBJECT

public:
    MocapPlugin(QWidget *parent = 0);

    ~MocapPlugin();

    /**
     * Load all configuration data for this panel from the given Config object.
     * @param config rviz config file
     */
    virtual void load(const rviz::Config &config);

    /**
     * Save all configuration data from this panel to the given
     * Config object.  It is important here that you call save()
     * on the parent class so the class id and panel name get saved.
     * @param config rviz config file
     */
    virtual void save(rviz::Config config) const;

    /**
     * This is the callback function for the marker positions,
     * depending on the cameraID and the cameras state, these are passed to the responsible Models
     * @param msg MarkerPosition message, containing seq, timestamp, position in image, cameraID
     */
    void pipe2function(const communication::MarkerPosition::ConstPtr& msg);

public Q_SLOTS:
    void initializeCameras();

    void streamCamera(int state);

    void changeID(int index);

    void videoCB(const sensor_msgs::ImageConstPtr& msg);

    void publishThreshold(int threshold);

private:
    void updateId(const std_msgs::Int32::ConstPtr &msg);

    ros::NodeHandle *nh;
    bool initialized = false;
    pair<uint, uint> currentID;
    ros::AsyncSpinner *spinner;
    ros::Publisher camera_control_pub, rviz_marker_pub, pose_pub;
    ros::Subscriber id_sub, video_sub, marker_position_sub;
    cv_bridge::CvImageConstPtr cv_ptr;
    Mat img;
    bool lockWhileWriting = false;
    Matrix4d ModelMatrix;
    map<int, CameraMarkerModel> camera;
    map<int, int> cameraState;
    vector<vector<float>> markerPositions;
};