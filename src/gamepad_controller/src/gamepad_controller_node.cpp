#include "gamepad_controller/gamepad_controller.h"

#include <ros/ros.h>

namespace
{
    const std::string path = "/dev/input/js0";
    const int fps = 25;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gamepad_controller");
    ros::NodeHandle nh;

    gamepad::GamepadController controller(&nh, ::path);
    ros::Rate loop_rate(fps);
    while (nh.ok())
    {
        controller.process();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

