#include "visionNode.hpp"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "VisionNode", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);	
    
	VisionNode visionNode;

    ros::Duration d(1.0);
    while (ros::ok()){
        d.sleep();
    }

    return 0;
}
