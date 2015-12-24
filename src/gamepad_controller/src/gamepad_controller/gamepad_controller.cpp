#include "gamepad_controller.h"

//msgs
#include "gamepad_controller/JsEvent.h"

//ros
#include <ros/ros.h>

//linux
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>

using gamepad::GamepadController;
using gamepad_controller::JsEvent;

class GamepadController::Impl
{
public:
    std::string device;
    int fd = -1;
    bool isOpened = false;
    ros::Publisher pub;

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
    d->pub = nh->advertise< JsEvent >("gamepad/event", 1000);
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
    JsEvent jsEvent;
    while (read(d->fd, &event, sizeof(event)) == sizeof(event))
    {
        memcpy(&jsEvent, &event + sizeof(event.time), sizeof(JsEvent));
        d->pub.publish(jsEvent);
    }
}
