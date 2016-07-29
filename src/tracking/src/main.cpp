// Include GLEW
#include <GL/glew.h>
// Include sfml
#include <SFML/Window.hpp>

#include "ncurses_tracking.hpp"

#define WINDOW_WIDTH 640
#define WINDOW_HEIGHT 480

using namespace std;

typedef enum
{
    Render,
    Publish
} ActionState;

static std::map<ActionState, std::string> state_strings = {
        { Render,                   "Render screen" },
        { Publish,                  "Publish the pose" }
};

ActionState NextState(ActionState s)
{
    ActionState newstate;
    switch (s)
    {
        case Render:
            newstate = Publish;
            break;
        case Publish:
            newstate = Render;
            break;
    }
    return newstate;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tracking_node");

    // create sfml window
    sf::ContextSettings settings;
    settings.depthBits = 24;
    settings.stencilBits = 8;
    sf::Window window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT, 32), "Marker Tracker", sf::Style::Titlebar | sf::Style::Close, settings);

    // Initialize GLEW
    glewExperimental = GL_TRUE;
    glewInit();

    NCurses_tracking ncurse(&window);

    char cmd;
    noecho();
    do{
        timeout(100);
        cmd = mvgetch(4,0);
        switch (cmd){
            case '0':
                ncurse.togglePosePublishing();
                break;
            case '1':
                ncurse.toggleVirtualMarker();
                break;
            case '2':
                ncurse.streamVideo();
                break;
            case 's':
                ncurse.saveCameraImage();
                break;
        }
        ncurse.showCameraInfo();
    }while( cmd != '9');
//
//    char c;
//    bool running = true;
//    while (running){
//        // handle events
//        sf::Event event;
//        while (window.pollEvent(event))
//        {
//            switch (event.type)
//            {
//                // window closed
//                case sf::Event::Closed:
//                    running = false;
//                    break;
//                case sf::Event::Resized:
//                    glViewport(0, 0, event.size.width, event.size.height);
//                    break;
//            }
//        }
//
//        switch(currentState)
//        {
//            case Render: {
//                pose_sphere = markerTracker.ModelMatrix.cast<float>();
//                model.updateViewMatrix(window);
//                // render the object with the updated modelmatrix
//                Mat img;
//                model.render(pose_sphere,img);
//                window.display();
//
//                currentState = NextState(currentState);
//                break;
//            }
//            case Publish: {
//                static uint next_id = 0;
//                visualization_msgs::Marker marker;
//                marker.header.frame_id = "map";
//                marker.header.stamp = ros::Time::now();
//                marker.header.seq = next_id++;
//                marker.ns = "markertracker";
//                marker.id = next_id;
//                marker.type = visualization_msgs::Marker::CUBE;
//                marker.action = visualization_msgs::Marker::ADD;
//                Vector3f position = pose_sphere.topRightCorner(3,1);
////                if(ros::Time::now().sec%10==0)
////                    printf("( %.4f, %.4f, %.4f )\n", position(0), position(1), position(2));
//                Matrix3f pose = pose_sphere.topLeftCorner(3,3);
//                Quaternionf q(pose);
//                marker.pose.position.x = position(0);
//                marker.pose.position.y = position(1);
//                marker.pose.position.z = position(2);
//                marker.pose.orientation.x = q.x();
//                marker.pose.orientation.y = q.y();
//                marker.pose.orientation.z = q.z();
//                marker.pose.orientation.w = q.w();
//                marker.scale.x = 0.01f;
//                marker.scale.y = 0.01f;
//                marker.scale.z = 0.01f;
//                marker.color.r = 0;
//                marker.color.g = 1;
//                marker.color.b = 0;
//                marker.color.a = 1;
//                marker.lifetime = ros::Duration(30);
//                rviz_marker_pub.publish(marker);
//                currentState = NextState(currentState);
//            }
//        }
//    }
//
//    window.close();

    return 0;
}
