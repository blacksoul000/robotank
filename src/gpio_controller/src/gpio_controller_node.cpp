#include "gpio_controller/gpio_controller.h"

#include <ros/ros.h>

namespace
{
    const int fps = 10;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gpio_controller");
    ros::NodeHandle nh;

    gpio::Controller controller(&nh);

    ros::Rate loop_rate(fps);
    while (nh.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        controller.process();
    }

    return 0;
}
