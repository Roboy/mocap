#include "markertracker.hpp"

using namespace std;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "tracking_node");

    MarkerTracker markerTracker;
    markerTracker.init();

    ros::spin();

    return 0;
}
