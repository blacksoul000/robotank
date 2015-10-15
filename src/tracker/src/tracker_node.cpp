#include "tracker/tracker.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;

    va::Tracker tracker(&nh);

    ros::spin();
    return 0;
}
