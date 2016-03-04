#include "robo_core/robo_core.h"

#include <ros/ros.h>

namespace
{
    const int fps = 100;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robo_core");
    ros::NodeHandle nh;

    robo_core::RoboCore core(&nh);
    ros::Rate loop_rate(fps);
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

