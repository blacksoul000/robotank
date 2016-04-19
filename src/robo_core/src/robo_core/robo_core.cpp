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
#include <limits>

using robo_core::RoboCore;

namespace
{
    const float defaultDotsPerDegree = 100;

    enum DeviceId
    {
        Chassis = 0,
        Tower = 1
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
    d->influenceChassis.deviceId = ::DeviceId::Chassis;
    d->influenceTower.bySpeed = true;
    d->influenceChassis.deviceId = ::DeviceId::Tower;

    d->trackerStatusS = nh->subscribe("tracker/status", 1,
                                      &RoboCore::Impl::onTrackerStatusChanged, d);
    d->trackerS = nh->subscribe("tracker/deviation", 1, &RoboCore::Impl::onTrackerDeviation, d);
    d->analog1S = nh->subscribe("gamepad/analog1", 20, &RoboCore::Impl::onChassisEvent, d);
    d->analog2S = nh->subscribe("gamepad/analog2", 20, &RoboCore::Impl::onTowerEvent, d);
    d->dpdS = nh->subscribe("camera/dotsPerDegree", 1, &RoboCore::Impl::onDotsPerDegreeChanged, d);

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
    const double value = this->smooth(event.value, SHRT_MAX);
    switch (event.number)
    {
    case 0x0: // 1st Axis X
        inf.x = value;
        break;
    case 0x1: // 1st Axis Y
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
    const double value = this->smooth(event.value, SHRT_MAX);
    switch (event.number)
    {
    case 0x2: // 2nd Axis X
        inf.x = value;
        break;
    case 0x5: // 2nd Axis Y
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
