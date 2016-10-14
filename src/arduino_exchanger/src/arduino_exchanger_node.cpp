#include "arduino_exchanger/arduino_exchanger.h"

#include <ros/ros.h>

namespace
{
    const std::string path = "/dev/ttyAMA0";
    const int fps = 100;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "arduino_exchanger");
    ros::NodeHandle nh;

    arduino::ArduinoExchanger controller(&nh, ::path);
    ros::Rate loop_rate(fps);
    while (nh.ok())
    {
        controller.process();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

