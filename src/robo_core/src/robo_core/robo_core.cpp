#include "robo_core.h"

//msgs
#include "gamepad_controller/JsEvent.h"
#include "tracker/PointF.h"
#include "video_source/PointF.h"
#include "std_msgs/UInt8.h"
#include "robo_core/Influence.h"
#include <sensor_msgs/Joy.h>

//ros
#include <ros/ros.h>

//linux
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <limits>

using robo_core::RoboCore;

namespace
{
    constexpr float defaultDotsPerDegree = 100;
    constexpr double influenceCoef = 90.0 / 32767;

    enum DeviceId
    {
        Chassis = 0,
        Tower = 1
    };

    enum Axes
    {
        X1 = 0,
        Y1 = 1,
        X2 = 2,
        Y2 = 5
    };
}

class RoboCore::Impl
{
public:
    State state = State::Search;
    robo_core::Influence influenceChassis;
    robo_core::Influence influenceTower;

    ros::Publisher influenceP;
    ros::Subscriber analog1S;
    ros::Subscriber analog2S;
    ros::Subscriber trackerS;
    ros::Subscriber trackerStatusS;
    ros::Subscriber dpdS;
    ros::Subscriber joyS;

    video_source::PointF dotsPerDegree;
    void onChassisEvent(const gamepad_controller::JsEvent& event);
    void onTowerEvent(const gamepad_controller::JsEvent& event);
    void onTrackerStatusChanged(const std_msgs::UInt8& status);
    void onTrackerDeviation(const tracker::PointF& deviation);
    void onDotsPerDegreeChanged(const video_source::PointF& dpd);
    void onJoyEvent(const sensor_msgs::Joy::ConstPtr& joy);

    double smooth(double value, double maxInputValue, double maxOutputValue) const;
};

RoboCore::RoboCore(ros::NodeHandle* nh):
    d(new Impl)
{
    d->influenceP = nh->advertise< robo_core::Influence >("core/influence", 10);
    d->influenceChassis.bySpeed = true;
    d->influenceChassis.deviceId = ::DeviceId::Chassis;
    d->influenceTower.bySpeed = true;
    d->influenceChassis.deviceId = ::DeviceId::Tower;

    d->trackerStatusS = nh->subscribe("tracker/status", 1,
                                      &RoboCore::Impl::onTrackerStatusChanged, d);
    d->trackerS = nh->subscribe("tracker/deviation", 1, &RoboCore::Impl::onTrackerDeviation, d);
    d->analog1S = nh->subscribe("gamepad/analog1", 100, &RoboCore::Impl::onChassisEvent, d);
    d->analog2S = nh->subscribe("gamepad/analog2", 100, &RoboCore::Impl::onTowerEvent, d);
    d->dpdS = nh->subscribe("camera/dotsPerDegree", 1, &RoboCore::Impl::onDotsPerDegreeChanged, d);
    d->joyS = nh->subscribe<sensor_msgs::Joy>("joy", 10, &RoboCore::Impl::onJoyEvent, d);

    ros::param::param< float >("camera/dotsPerDegreeH", d->dotsPerDegree.x, ::defaultDotsPerDegree);
    ros::param::param< float >("camera/dotsPerDegreeV", d->dotsPerDegree.y, ::defaultDotsPerDegree);
}

RoboCore::~RoboCore()
{
    delete d;
}

//------------------------------------------------------------------------------------
void RoboCore::Impl::onJoyEvent(const sensor_msgs::Joy::ConstPtr& joy)
{
    short x1 = -this->smooth(joy->axes[Axes::X1], 1, SHRT_MAX);
    short y1 = this->smooth(joy->axes[Axes::Y1], 1, SHRT_MAX);
    short x2 = -this->smooth(joy->axes[Axes::X2], 1, SHRT_MAX);
    short y2 = this->smooth(joy->axes[Axes::Y2], 1, SHRT_MAX);
    if (influenceChassis.x != x1 || influenceChassis.y != y1)
    {
        ROS_WARN("onJoyEvent chassis: %d %d", x1, y1);
        influenceChassis.x = x1;
        influenceChassis.y = y1;
        influenceP.publish(influenceChassis);
    }
    if (influenceTower.x != x2 || influenceTower.y != y2)
    {
        ROS_WARN("onJoyEvent tower: %d %d", x2, y2);
        influenceTower.x = x2;
        influenceTower.y = y2;
        influenceP.publish(influenceTower);
    }
}

void RoboCore::Impl::onChassisEvent(const gamepad_controller::JsEvent& event)
{
    ROS_WARN("onJsAnalog1: type = %d, number = %d, value = %d", event.type, event.number, event.value);

    auto& inf = influenceChassis;
    const double value = this->smooth(event.value, SHRT_MAX, SHRT_MAX);

    switch (event.number)
    {
    case Axes::X1:
        inf.x = value;
        break;
    case Axes::Y1:
        inf.y = value;
        break;
    default:
        return;
    }
    influenceP.publish(inf);
}

void RoboCore::Impl::onTowerEvent(const gamepad_controller::JsEvent& event)
{
    if (state != State::Search) return;
    ROS_WARN("onJsAnalog2: type = %d, number = %d, value = %d", event.type, event.number, event.value);
    auto& inf = influenceTower;
    const double value = this->smooth(event.value, SHRT_MAX, SHRT_MAX);
    switch (event.number)
    {
    case Axes::X2:
        inf.x = value;
        break;
    case Axes::Y2:
        inf.y = value;
        break;
    default:
        return;
    }
    influenceP.publish(inf);
}

void RoboCore::Impl::onTrackerDeviation(const tracker::PointF& deviation)
{
    ROS_WARN("onTrackerDeviation: x = %f, y = %f", deviation.x, deviation.y);
    robo_core::Influence msg;
    msg.bySpeed = false;
    msg.x = (deviation.x / dotsPerDegree.x) / ::influenceCoef;
    msg.y = (deviation.y / dotsPerDegree.y) / ::influenceCoef;
    influenceP.publish(msg);
}

void RoboCore::Impl::onTrackerStatusChanged(const std_msgs::UInt8& status)
{
    (status.data == 1) ? state = State::Track : State::Search;
}

double RoboCore::Impl::smooth(double value, double maxInputValue, double maxOutputValue) const
{
    return pow((value / maxInputValue), 3) * maxOutputValue;
}

void RoboCore::Impl::onDotsPerDegreeChanged(const video_source::PointF& dpd)
{
    dotsPerDegree = dpd;
}
