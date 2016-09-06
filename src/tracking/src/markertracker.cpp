#include "markertracker.hpp"

MarkerTracker::MarkerTracker(){
    spinner = new ros::AsyncSpinner(5);
    spinner->start();
    marker_position_sub = nh.subscribe("/raspicamera/marker_position", 1, &MarkerTracker::pipe2function, this);
    rviz_marker_pub=nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
    camera_control_pub=nh.advertise<communication::CameraControl>("/camera_control", 1000);
    video_sub=nh.subscribe("/raspicamera/video", 1, &MarkerTracker::videoCB, this);
}

MarkerTracker::~MarkerTracker() {
    spinner->stop();
    delete spinner;
}

bool MarkerTracker::init() {
    ROS_INFO("please stand in front of cameras");
    initialized = false;
    ros::Duration d(1.0);
    while(!initialized)
        d.sleep();
    return true;
}

void MarkerTracker::pipe2function(const communication::MarkerPosition::ConstPtr& msg){
    // check if camera was already registered
    auto it = camera.find(msg->cameraID);
    if(it != camera.end()) // camera already registered
    {
        camera[msg->cameraID].markerVisible = msg->markerVisible;
        camera[msg->cameraID].fps = msg->fps;
        switch(cameraState[msg->cameraID]){
            case Uninitialized:{
                cameraState[msg->cameraID] = it->second.initializeModel(msg);
                break;
            }
            case Initialized:{
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
            case Tracking:{
                cameraState[msg->cameraID] = it->second.track(msg);
                if(cameraState[msg->cameraID]!=Error) {
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
            case Error:{
                cameraState[msg->cameraID] = Tracking; // TODO: error handling
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
        cameraState[msg->cameraID] = Uninitialized;
    }
}

bool MarkerTracker::sendCameraControl(uint ID, uint control, bool value){
    // check if camera exists
    auto it = camera.find(ID);
    if(it != camera.end())
    {
        communication::CameraControl msg;
        msg.cameraID = ID;
        msg.control = control;
        switch(control) {
            case toggleVideoStream:
                msg.value1 = value;
                break;
            default:
                return false;
        }
        camera_control_pub.publish(msg);
    }
    return false;
}

void MarkerTracker::videoCB(const sensor_msgs::ImageConstPtr& msg){
    lockWhileWriting = true;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    lockWhileWriting = false;
}