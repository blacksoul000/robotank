#include "gamepad_controller.h"

//ros
#include <ros/ros.h>
//#include "std_msgs/UInt8.h"

//linux
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>

using gamepad::GamepadController;

class GamepadController::Impl
{
public:
    std::string device;
    int fd = -1;
    bool isOpened = false;

    bool open()
    {
        fd = ::open(device.c_str(), O_RDONLY | O_NONBLOCK);
        isOpened = (fd >= 0);
        return isOpened;
    }
};

GamepadController::GamepadController(ros::NodeHandle* nh, const std::string& path):
    d(new Impl)
{
    d->device = path;
}

GamepadController::~GamepadController()
{
    close(d->fd);
    delete d;
}

void GamepadController::process()
{
    if (!d->isOpened) d->open();

    struct js_event event;
    while (read(d->fd, &event, sizeof(event)) == sizeof(event))
    {
        switch (event.type) {
        case JS_EVENT_AXIS:
            break;
        case JS_EVENT_BUTTON:
            if (event.value)
            {
                ROS_WARN("pressed %d", event.number);
            }
            else
            {
                ROS_WARN("released %d", event.number);
            }
            break;
        default:
            break;
        }
    }
}
