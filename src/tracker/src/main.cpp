#include "tracker_node.h"

#include <ros/ros.h>
namespace
{
    const int fps = 10;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;

    va::TrackerNode node(&nh);

    ros::Rate loop_rate(fps);
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
