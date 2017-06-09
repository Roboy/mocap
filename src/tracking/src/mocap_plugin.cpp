#include "mocap_plugin.hpp"

MocapPlugin::MocapPlugin(QWidget *parent)
        : rviz::Panel(parent) {
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "MocapRvizPlugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;

    spinner = new ros::AsyncSpinner(3);

    camera_control_pub = nh->advertise<communication::CameraControl>("/mocap/camera_control", 100);
    id_sub = nh->subscribe("/mocap/cameraID", 100, &MocapPlugin::updateId, this);
    video_sub = nh->subscribe("/mocap/video", 1, &MocapPlugin::videoCB, this);
    marker_position_sub = nh->subscribe("/mocap/marker_position", 100, &MocapPlugin::pipe2function, this);
    rviz_marker_pub = nh->advertise<visualization_msgs::Marker>("/visualization_marker", 100);
    pose_pub = nh->advertise<geometry_msgs::Pose>("/mocap/MarkerPose", 1);

    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

    QComboBox *cameraIDs = new QComboBox();
    cameraIDs->setObjectName("cameraIDs");
    connect(cameraIDs, SIGNAL(currentIndexChanged(int)), this, SLOT(changeID(int)));
    frameLayout->addWidget(cameraIDs);

    QVBoxLayout *cameraStatusLayout = new QVBoxLayout();

    LightWidget *mocapStatus = new LightWidget();
    mocapStatus->setObjectName("mocapStatus");
    cameraStatusLayout->addWidget(mocapStatus);

    QSlider *slider = new QSlider(Qt::Horizontal, this);
    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(publishThreshold(int)));
    slider->setTickInterval(1);
    slider->setRange(0, 255);
    slider->setSliderPosition(240);
    frameLayout->addWidget(slider);

    QTableWidget *table = new QTableWidget(3, 2);
    table->setObjectName("table");
    QTableWidgetItem *item0 = new QTableWidgetItem("marker visible");
    QTableWidgetItem *item1 = new QTableWidgetItem("frames per second");
    QTableWidgetItem *item2 = new QTableWidgetItem("reprojection error");
    table->setItem(0, 0, item0);
    table->setItem(1, 0, item1);
    table->setItem(2, 0, item2);
    cameraStatusLayout->addWidget(table);

    frameLayout->addLayout(cameraStatusLayout);

    QPushButton *initcameras = new QPushButton(tr("initialize cameras"));
    initcameras->setObjectName("initwalkcontroller");
    connect(initcameras, SIGNAL(clicked()), this, SLOT(initializeCameras()));
    frameLayout->addWidget(initcameras);

    QCheckBox *streamcamera = new QCheckBox(tr("stream camera"));
    connect(streamcamera, SIGNAL(stateChanged(int)), this, SLOT(streamCamera(int)));
    frameLayout->addWidget(streamcamera);

    QLabel *camimage = new QLabel(tr("camimage"));
    camimage->setObjectName("camimage");
    frameLayout->addWidget(camimage);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);

    // Add the frame to the main layout
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);
}

MocapPlugin::~MocapPlugin() {
    delete nh;
    delete spinner;
}

void MocapPlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
}

void MocapPlugin::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void MocapPlugin::pipe2function(const communication::MarkerPosition::ConstPtr &msg) {
    // check if camera was already registered
    auto it = camera.find(msg->cameraID);
    if (it != camera.end()) // camera already registered
    {
        LightWidget *mocapStatus = this->findChild<LightWidget *>("mocapStatus");
        camera[msg->cameraID].markerVisible = msg->markerVisible;
        camera[msg->cameraID].fps = msg->fps;
        QTableWidget *table = this->findChild<QTableWidget *>("table");
        QTableWidgetItem *item0 = new QTableWidgetItem(QString::number(msg->markerVisible));
        QTableWidgetItem *item1 = new QTableWidgetItem(QString::number(msg->fps));
        table->setItem(0, 1, item0);
        table->setItem(1, 1, item1);
        switch (cameraState[msg->cameraID]) {
            case CameraState::Uninitialized: {
                mocapStatus->setColor(Qt::gray);
                cameraState[msg->cameraID] = it->second.initializeModel(msg);
                QTableWidgetItem *item2 = new QTableWidgetItem(QString::number(camera[msg->cameraID]
                                                                                       .reprojectionError));
                table->setItem(2, 1, item2);
                break;
            }
            case CameraState::Initialized: {
                mocapStatus->setColor(Qt::blue);
                bool allInitialized = true;
                for (auto state = cameraState.begin(); state != cameraState.end(); ++state) {
                    if (state->second == Uninitialized)
                        allInitialized = false;
                }
                if (allInitialized) { // if all cameras are initialized, set initialized flag and cameraState to Tracking
                    initialized = true;
                    for (auto state = cameraState.begin(); state != cameraState.end(); ++state) {
                        state->second = Tracking; // TODO: possibly find trafo between cameras here
                    }
                } else {
                    initialized = false;
                }
                break;
            }
            case CameraState::Tracking: {
                mocapStatus->setColor(Qt::green);
                cameraState[msg->cameraID] = it->second.track(msg);
                QTableWidgetItem *item2 = new QTableWidgetItem(QString::number(camera[msg->cameraID]
                                                                                       .reprojectionError));
                table->setItem(2, 1, item2);
                if (cameraState[msg->cameraID] != CameraState::Error && msg->cameraID == currentID.second) {
                    static bool add = true;
                    visualization_msgs::Marker mesh;
                    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                    mesh.header.frame_id = "world";
                    mesh.ns = "markerModel";
                    mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
                    mesh.color.r = 1.0f;
                    mesh.color.g = 1.0f;
                    mesh.color.b = 1.0f;
                    mesh.color.a = 0.5;
                    mesh.scale.x = 1.0;
                    mesh.scale.y = 1.0;
                    mesh.scale.z = 1.0;
                    mesh.lifetime = ros::Duration();
                    mesh.action = visualization_msgs::Marker::ADD;
                    mesh.header.stamp = ros::Time::now();
                    mesh.id = 80;
                    Matrix3d pose = camera[msg->cameraID].ModelMatrix.topLeftCorner(3, 3);
                    Quaterniond q(pose);
                    Vector3d position = camera[msg->cameraID].ModelMatrix.topRightCorner(3, 1);
                    mesh.pose.position.x = position(0);
                    mesh.pose.position.y = position(1);
                    mesh.pose.position.z = position(2);
                    q.normalize();
                    mesh.pose.orientation.x = q.x();
                    mesh.pose.orientation.y = q.y();
                    mesh.pose.orientation.z = q.z();
                    mesh.pose.orientation.w = q.w();
                    mesh.mesh_resource = "package://tracking_node/models/markermodel.STL";
                    rviz_marker_pub.publish(mesh);

                    geometry_msgs::Pose pose_msg;
                    pose_msg.orientation = mesh.pose.orientation;
                    pose_msg.position = mesh.pose.position;
                    pose_pub.publish(pose_msg);
                }
                break;
            }
            case CameraState::Error: {
                mocapStatus->setColor(Qt::yellow);
                cameraState[msg->cameraID] = CameraState::Tracking; // TODO: error handling
                break;
            }
        }
        mocapStatus->repaint();
    } else { // register new camera
        camera[msg->cameraID].id = msg->cameraID;
        sprintf(camera[msg->cameraID].name, "camera %d", msg->cameraID);
        cameraState[msg->cameraID] = CameraState::Uninitialized;
    }
}

void MocapPlugin::initializeCameras() {
    for (auto it = cameraState.begin(); it != cameraState.end(); it++) {
        it->second = Uninitialized;
    }
}

void MocapPlugin::streamCamera(int state) {
    communication::CameraControl msg;
    msg.cameraID = currentID.second;
    msg.control = toggleVideoStream;
    if (state == Qt::Checked)
        msg.boolValue = true;
    else if (state == Qt::Unchecked)
        msg.boolValue = false;
    camera_control_pub.publish(msg);
}

void MocapPlugin::changeID(int index) {
    QComboBox *cameraIDs = this->findChild<QComboBox *>("cameraIDs");
    currentID = make_pair(index, cameraIDs->currentText().toInt());
}

void MocapPlugin::videoCB(const sensor_msgs::ImageConstPtr &msg) {
    lockWhileWriting = true;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        cv_ptr->image.copyTo(img);
        flip(img,img,1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    lockWhileWriting = false;

    char str[1];
    for (uint col = 0; col < MARKER; col++) {
        cv::Point2f center(camera[currentID.second].projectedPosition2D(0, col),
                           camera[currentID.second].projectedPosition2D(1, col));
        circle(img, center, 5, cv::Scalar(255, 255, 0), 4);
        line(img, cv::Point2f(camera[currentID.second].origin2D(0), camera[currentID.second].origin2D(1)), center,
             cv::Scalar::all(255), 4);
        sprintf(str, "%d", camera[currentID.second].markerIDs[col]);
        putText(img, str, center, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar::all(255));
    }

    QLabel *camimage = this->findChild<QLabel *>("camimage");
    int w = img.cols;
    int h = img.rows;
    QImage qim(w, h, QImage::Format_RGB32);
    QRgb pixel;
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            int gray = (int) img.at < unsigned char > (j, i);
            pixel = qRgb(gray, gray, gray);
            qim.setPixel(i, j, pixel);
        }
    }
    QPixmap pixmap = QPixmap::fromImage(qim);
    camimage->setPixmap(pixmap);
    camimage->repaint();
}

void MocapPlugin::updateId(const std_msgs::Int32::ConstPtr &msg) {
    QComboBox *cameraIDs = this->findChild<QComboBox *>("cameraIDs");
    int index = cameraIDs->findText(QString::number(msg->data));
    if (index == -1) {
        cameraIDs->addItem(QString::number(msg->data));
        cameraIDs->repaint();
    }
}

void MocapPlugin::publishThreshold(int threshold) {
    communication::CameraControl msg;
    msg.cameraID = currentID.second;
    msg.control = changeThreshold;
    msg.intValue = threshold;
    camera_control_pub.publish(msg);
}
PLUGINLIB_EXPORT_CLASS(MocapPlugin, rviz::Panel)