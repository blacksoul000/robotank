#include "gpio_controller.h"

//msgs
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include "std_msgs/UInt8.h"

//ros
#include <ros/ros.h>

#include <wiringPi.h>

using gpio::Controller;

namespace
{
    const uint8_t shotFinishedPin1 = 28;
    const uint8_t shotFinishedPin2 = 29;

    const uint8_t resetPin = 4;
}

class Controller::Impl
{
public:
    ros::Publisher shotStatusPub;
    std::list< ros::Subscriber > subscribers;
    bool shoting = false;
    bool shotClosing = false;

    void onShotStatusChanged(bool shot);
    void onJoyEvent(const sensor_msgs::Joy::ConstPtr& joy);
    void onResetArduinoRequest(std_msgs::Empty);
};

Controller::Controller(ros::NodeHandle* nh):
    d(new Impl)
{
    d->shotStatusPub = nh->advertise< std_msgs::UInt8 >("core/shot", 1);

    d->subscribers.push_back(nh->subscribe("joy", 1, &Controller::Impl::onJoyEvent, d));
    d->subscribers.push_back(nh->subscribe("arduino/reset", 1,
                                           &Controller::Impl::onResetArduinoRequest, d));

    wiringPiSetup();

    pinMode (::shotFinishedPin1, OUTPUT);
    pinMode (::shotFinishedPin2, INPUT);

    d->onShotStatusChanged(false);
}

Controller::~Controller()
{
    digitalWrite (::shotFinishedPin1, LOW);
    delete d;
}

void Controller::process()
{
    if (!d->shoting) return;

    if(digitalRead(::shotFinishedPin2) == HIGH)
    {
        d->shotClosing = true;
    } else if (d->shotClosing)
    {
        d->shotClosing = false;
        d->onShotStatusChanged(false);
    }
}

//------------------------------------------------------------------------------------
void Controller::Impl::onJoyEvent(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (shoting) return;

    if (joy->buttons[6] == 1 && joy->buttons[7] == 1) // both triggers
    {
        this->onShotStatusChanged(true);
    }
}

void Controller::Impl::onResetArduinoRequest(std_msgs::Empty)
{
    digitalWrite (::resetPin, HIGH);
    delay(50);
    digitalWrite (::resetPin, LOW);
}

void Controller::Impl::onShotStatusChanged(bool shot)
{
    ROS_WARN("onShotStatusChanged(%d)", shot);
    shoting = shot;
    std_msgs::UInt8 msg;
    msg.data = shot;
    shotStatusPub.publish(msg);
    digitalWrite (::shotFinishedPin1, shot ? HIGH : LOW);
}
