#include "tracker_node.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;

    va::TrackerNode node(&nh);

    ros::spin();
    return 0;
}
