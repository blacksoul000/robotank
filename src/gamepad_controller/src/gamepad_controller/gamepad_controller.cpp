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
    ros::Publisher analog1P;
    ros::Publisher analog2P;
    ros::Publisher digitalP;
    ros::Publisher buttonsP;
    ros::Publisher triggersP;

    bool open()
    {
        fd = ::open(device.c_str(), O_RDONLY | O_NONBLOCK);
        isOpened = (fd >= 0);
        if (!isOpened)
        {
            ROS_DEBUG_ONCE("Failed to open %s", device.c_str());
        }
        return isOpened;
    }
};

GamepadController::GamepadController(ros::NodeHandle* nh, const std::string& path):
    d(new Impl)
{
    d->device = path;
//    d->statusP = nh->advertise< JsEvent >("gamepad/status", 1000);
    d->analog1P = nh->advertise< JsEvent >("gamepad/analog1", 50);
    d->analog2P = nh->advertise< JsEvent >("gamepad/analog2", 50);
    d->digitalP = nh->advertise< JsEvent >("gamepad/digatal", 10);
    d->buttonsP = nh->advertise< JsEvent >("gamepad/buttons", 20);
    d->triggersP = nh->advertise< JsEvent >("gamepad/triggers", 50);
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
        memcpy(&jsEvent, &event.value, sizeof(JsEvent));
        switch (event.type)
        {
        case JS_EVENT_BUTTON:
            d->buttonsP.publish(jsEvent);
            break;
        case JS_EVENT_AXIS:
            if (jsEvent.number == 0 || jsEvent.number == 1)
            {
                d->analog1P.publish(jsEvent);
            } else if (jsEvent.number == 2 || jsEvent.number == 5)
            {
                d->analog2P.publish(jsEvent);
//              via bt I get many "spam" events with numbers 6,7,8, so disabling
//            } else if (jsEvent.number == 6 || jsEvent.number == 7) //usb connection
            } else if (jsEvent.number == 9 || jsEvent.number == 10) //ds4drv(bluetooth) connection
            {
                d->digitalP.publish(jsEvent);
            } else if (jsEvent.number == 3 || jsEvent.number == 4)
            {
                d->triggersP.publish(jsEvent);
            }
            break;
        default:
            break;
        }
    }
}
