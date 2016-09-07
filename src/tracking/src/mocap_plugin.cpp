#include "mocap_plugin.hpp"

MocapPlugin::MocapPlugin(QWidget *parent)
        : rviz::Panel(parent){
    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;

    // Create the frame to hold all the widgets
    QFrame *mainFrame = new QFrame();

    QVBoxLayout *frameLayout = new QVBoxLayout();

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

    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "MocapRvizPlugin",
                  ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    }

    nh = new ros::NodeHandle;

    spinner = new ros::AsyncSpinner(3);

    camera_control_pub = nh->advertise<communication::CameraControl>("/mocap/camera_control", 1);
    id_sub = nh->subscribe("/mocap/id", 1, &MocapPlugin::updateId, this);
    video_sub = nh->subscribe("/mocap/video", 1, &MocapPlugin::videoCB, this);
    marker_position_sub = nh->subscribe("/mocap/marker_position", 1, &MocapPlugin::pipe2function, this);
    rviz_marker_pub=nh->advertise<visualization_msgs::Marker>("/visualization_marker", 1);
}

MocapPlugin::~MocapPlugin(){
    delete nh;
    delete spinner;
}
void MocapPlugin::load(const rviz::Config &config) {
    rviz::Panel::load(config);
}

void MocapPlugin::save(rviz::Config config) const {
    rviz::Panel::save(config);
}

void MocapPlugin::pipe2function(const communication::MarkerPosition::ConstPtr& msg){
    // check if camera was already registered
    auto it = camera.find(msg->cameraID);
    if(it != camera.end()) // camera already registered
    {
        camera[msg->cameraID].markerVisible = msg->markerVisible;
        camera[msg->cameraID].fps = msg->fps;
        switch(cameraState[msg->cameraID]){
            case CameraState::Uninitialized:{
                cameraState[msg->cameraID] = it->second.initializeModel(msg);
                break;
            }
            case CameraState::Initialized:{
                bool allInitialized = true;
                for(auto state = cameraState.begin(); state != cameraState.end(); ++state){
                    if(state->second==Uninitialized)
                        allInitialized = false;
                }
                if(allInitialized) { // if all cameras are initialized, set initialized flag and cameraState to Tracking
                    initialized = true;
                    for(auto state = cameraState.begin(); state != cameraState.end(); ++state) {
                        state->second = Tracking; // TODO: possibly find trafo between cameras here
                    }
                }else{
                    initialized = false;
                }
                break;
            }
            case CameraState::Tracking:{
                cameraState[msg->cameraID] = it->second.track(msg);
                if(cameraState[msg->cameraID]!=CameraState::Error) {
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
                    if (add) {
                        mesh.action = visualization_msgs::Marker::ADD;
                        add = false;
                    } else {
                        mesh.action = visualization_msgs::Marker::MODIFY;
                    }
                    mesh.header.stamp = ros::Time::now();
                    mesh.id = 80;
                    Matrix3d pose = camera[msg->cameraID].ModelMatrix.topLeftCorner(3,3);
                    Quaterniond q(pose);
                    Vector3d position = camera[msg->cameraID].ModelMatrix.topRightCorner(3,1);
                    mesh.pose.position.x = position(0);
                    mesh.pose.position.y = position(1);
                    mesh.pose.position.z = position(2);
                    mesh.pose.orientation.x = q.x();
                    mesh.pose.orientation.y = q.y();
                    mesh.pose.orientation.z = q.z();
                    mesh.pose.orientation.w = q.w();
                    mesh.mesh_resource = "package://tracking_node/models/markermodel.dae";
                    rviz_marker_pub.publish(mesh);
                }
                break;
            }
            case CameraState::Error:{
                cameraState[msg->cameraID] = CameraState::Tracking; // TODO: error handling
                break;
            }
        }
        markerPositions.clear();
        for(uint i = 0; i < msg->marker_position.size(); i++){
            vector<float> position;
            position.push_back(msg->marker_position[i].x);
            position.push_back(msg->marker_position[i].y);
            markerPositions.push_back(position);
        }
    }else{ // register new camera
        camera[msg->cameraID].id = msg->cameraID;
        sprintf(camera[msg->cameraID].name, "camera %d", msg->cameraID);
        cameraState[msg->cameraID] = CameraState::Uninitialized;
    }
}

void MocapPlugin::sendCameraControl(const communication::CameraControl::ConstPtr &msg){
    // check if camera exists
    auto it = camera.find(msg->cameraID);
    if(it != camera.end()){
        camera_control_pub.publish(msg);
    }
}

void MocapPlugin::initializeCameras(){

}

void MocapPlugin::streamCamera(int state){
    communication::CameraControl msg;
    msg.cameraID = 0;
    msg.control = toggleVideoStream;
    if(state == Qt::Checked)
        msg.value1 = true;
    else if(state == Qt::Unchecked)
        msg.value1 = false;
    ROS_INFO("stream camera %d", state);
    camera_control_pub.publish(msg);
}

void MocapPlugin::updateId(const communication::MarkerPosition::ConstPtr &msg){
    QComboBox* roboyID = this->findChild<QComboBox*>("roboyID");
    int index = roboyID->findText(QString::number(msg->cameraID));
    if(index==-1) {
        roboyID->addItem(QString::number(msg->cameraID));
        roboyID->repaint();
    }
}

void MocapPlugin::videoCB(const sensor_msgs::ImageConstPtr& msg){
    lockWhileWriting = true;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
        cv_ptr->image.copyTo(img);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    lockWhileWriting = false;
    QLabel* camimage = this->findChild<QLabel*>("camimage");
    int w=img.cols;
    int h=img.rows;
    QImage qim(w,h,QImage::Format_RGB32);
    QRgb pixel;
    for(int i=0;i<w;i++){
        for(int j=0;j<h;j++){
            int gray = (int)img.at<unsigned char>(j, i);
            pixel = qRgb(gray,gray,gray);
            qim.setPixel(i,j,pixel);
        }
    }
    QPixmap pixmap = QPixmap::fromImage(qim);
    camimage->setPixmap(pixmap);
    camimage->repaint();
}


PLUGINLIB_EXPORT_CLASS(MocapPlugin, rviz::Panel)