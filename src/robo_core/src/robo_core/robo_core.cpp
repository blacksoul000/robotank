#include "robo_core.h"

//msgs
#include "gamepad_controller/JsEvent.h"
#include "tracker/PointF.h"
#include "video_source/PointF.h"
#include "std_msgs/UInt8.h"
#include "robo_core/Influence.h"

//ros
#include <ros/ros.h>

//linux
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using robo_core::RoboCore;

namespace
{
    const float defaultDotsPerDegree = 100;
}

class RoboCore::Impl
{
public:
    State state = State::Search;
    robo_core::Influence influenceChassis;
    robo_core::Influence influenceTower;

    ros::Publisher influenceP;
    video_source::PointF dotsPerDegree;
    void onChassisEvent(const gamepad_controller::JsEvent& event);
    void onTowerEvent(const gamepad_controller::JsEvent& event);
    void onTrackerStatusChanged(const std_msgs::UInt8& status);
    void onTrackerDeviation(const tracker::PointF& deviation);
    void onDotsPerDegreeChanged(const video_source::PointF& dpd);

    double smooth(double value, double maxValue) const;
};

RoboCore::RoboCore(ros::NodeHandle* nh):
    d(new Impl)
{
    d->influenceP = nh->advertise< robo_core::Influence >("core/influence", 10);
    d->influenceChassis.bySpeed = true;
    d->influenceTower.bySpeed = true;

    nh->subscribe("tracker/status", 1, &RoboCore::Impl::onTrackerStatusChanged, d);
    nh->subscribe("tracker/deviation", 1, &RoboCore::Impl::onTrackerDeviation, d);
    nh->subscribe("gamepad/analog1", 20, &RoboCore::Impl::onChassisEvent, d);
    nh->subscribe("gamepad/analog2", 20, &RoboCore::Impl::onTowerEvent, d);
    nh->subscribe("camera/dotsPerDegree", 1, &RoboCore::Impl::onDotsPerDegreeChanged, d);

    ros::param::param< float >("camera/dotsPerDegreeH", d->dotsPerDegree.x, ::defaultDotsPerDegree);
    ros::param::param< float >("camera/dotsPerDegreeV", d->dotsPerDegree.y, ::defaultDotsPerDegree);
}

RoboCore::~RoboCore()
{
    delete d;
}

//------------------------------------------------------------------------------------
void RoboCore::Impl::onChassisEvent(const gamepad_controller::JsEvent& event)
{
    ROS_WARN("onJsAnalog1: type = %d, number = %d, value = %d", event.type, event.number, event.value);

    auto& inf = influenceChassis;
    switch (event.number)
    {
    case 0x0: // 2nd Axis X
        inf.x = event.value;
        break;
    case 0x1: // 2nd Axis Y
        inf.y = event.value;
        break;
    default:
        return;
    }
    influenceP.publish(inf);
}

void RoboCore::Impl::onTowerEvent(const gamepad_controller::JsEvent& event)
{
    if (state != State::Search) return;
    ROS_WARN("onJsAnalog1: type = %d, number = %d, value = %d", event.type, event.number, event.value);
    auto& inf = influenceTower;
    switch (event.number)
    {
    case 0x2: // 2nd Axis X
        inf.x = event.value;
        break;
    case 0x3: // 2nd Axis Y
        inf.y = event.value;
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
    msg.x = deviation.x / dotsPerDegree.x;
    msg.y = deviation.y / dotsPerDegree.y;
    influenceP.publish(msg);
}

void RoboCore::Impl::onTrackerStatusChanged(const std_msgs::UInt8& status)
{
    (status.data == 1) ? state = State::Track : State::Search;
}

double RoboCore::Impl::smooth(double value, double maxValue) const
{
    return pow((value / maxValue), 3);
}

void RoboCore::Impl::onDotsPerDegreeChanged(const video_source::PointF& dpd)
{
    dotsPerDegree = dpd;
}
