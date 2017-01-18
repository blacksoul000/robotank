#include "robo_core.h"

//msgs
#include "tracker/PointF.h"
#include "video_source/PointF.h"
#include "std_msgs/UInt8.h"
#include "robo_core/Influence.h"
#include "robo_core/PointF.h"
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

    template < class T >
    inline T bound(T minValue, T value, T maxValue)
    {
        return std::min(std::max(value, minValue), maxValue);
    }
}

class RoboCore::Impl
{
public:
    State state = State::Search;
    robo_core::Influence influence;
    video_source::PointF dotsPerDegree;

    short enginePowerLeft = SHRT_MAX;
    short enginePowerRight = SHRT_MAX;

    ros::Publisher influenceP;

    std::list< ros::Subscriber > subscribers;

    void onTrackerStatusChanged(const std_msgs::UInt8& status);
    void onTrackerDeviation(const tracker::PointF& deviation);
    void onDotsPerDegreeChanged(const video_source::PointF& dpd);
    void onEnginePowerChanged(const robo_core::PointF& enginePower);
    void onJoyEvent(const sensor_msgs::Joy::ConstPtr& joy);
    void onFireStatusChanged(const std_msgs::UInt8& status);

    double smooth(double value, double maxInputValue, double maxOutputValue) const;
    void setState(State state);
};

RoboCore::RoboCore(ros::NodeHandle* nh):
    d(new Impl)
{
    d->influenceP = nh->advertise< robo_core::Influence >("core/influence", 10);

    d->subscribers.push_back(nh->subscribe("tracker/status", 1,
                                           &RoboCore::Impl::onTrackerStatusChanged, d));
    d->subscribers.push_back(nh->subscribe("tracker/deviation", 1,
                                           &RoboCore::Impl::onTrackerDeviation, d));
    d->subscribers.push_back(nh->subscribe("camera/dotsPerDegree", 1,
                                           &RoboCore::Impl::onDotsPerDegreeChanged, d));
    d->subscribers.push_back(nh->subscribe("joy", 10,
                                           &RoboCore::Impl::onJoyEvent, d));
    d->subscribers.push_back(nh->subscribe("core/enginePower", 1,
                                           &RoboCore::Impl::onEnginePowerChanged, d));
    d->subscribers.push_back(nh->subscribe("core/shot", 1,
                                           &RoboCore::Impl::onFireStatusChanged, d));

    ros::param::param< float >("camera/dotsPerDegreeH", d->dotsPerDegree.x, ::defaultDotsPerDegree);
    ros::param::param< float >("camera/dotsPerDegreeV", d->dotsPerDegree.y, ::defaultDotsPerDegree);

    d->setState(d->state);
}

RoboCore::~RoboCore()
{
    delete d;
}

//------------------------------------------------------------------------------------
void RoboCore::Impl::onJoyEvent(const sensor_msgs::Joy::ConstPtr& joy)
{
    if (state == State::Search)
    {
        short x = this->smooth(joy->axes[Axes::X2], 1, SHRT_MAX);
        short y = this->smooth(joy->axes[Axes::Y2], 1, SHRT_MAX);
        influence.gunV = influence.cameraV = y;
        influence.towerH = x;
    }

    const float speed = joy->axes[Axes::Y1];
    const float turnSpeed = joy->axes[Axes::X1];

    influence.leftEngine = this->smooth(::bound< float >(-1, speed - turnSpeed, 1), 1, enginePowerLeft);
    influence.rightEngine = this->smooth(::bound< float >(-1, speed + turnSpeed, 1), 1, enginePowerRight);
    influenceP.publish(influence);

    if (state == State::Track)
    {
        influence.gunV = influence.cameraV = 0;
    }
}

void RoboCore::Impl::onTrackerDeviation(const tracker::PointF& deviation)
{
    if (state != State::Track) return;
    ROS_WARN("onTrackerDeviation: x = %f, y = %f", deviation.x, deviation.y);
    influence.towerH = (deviation.x / dotsPerDegree.x) / ::influenceCoef; // TODO PID
    influence.gunV = influence.cameraV = (deviation.y / dotsPerDegree.y) / ::influenceCoef;
    influenceP.publish(influence);
}

void RoboCore::Impl::onEnginePowerChanged(const robo_core::PointF& enginePower)
{
    ROS_WARN("onEnginePowerChanged n: x = %f, y = %f", enginePower.x, enginePower.y);
    enginePowerLeft = enginePower.x * SHRT_MAX / 100.0;
    enginePowerRight = enginePower.y * SHRT_MAX / 100.0;
}

void RoboCore::Impl::onTrackerStatusChanged(const std_msgs::UInt8& status)
{
    this->setState((status.data == 1) ? State::Track : State::Search);
}

void RoboCore::Impl::onFireStatusChanged(const std_msgs::UInt8& status)
{
    influence.shot = status.data;
}

double RoboCore::Impl::smooth(double value, double maxInputValue, double maxOutputValue) const
{
    return pow((value / maxInputValue), 3) * maxOutputValue;
}

void RoboCore::Impl::onDotsPerDegreeChanged(const video_source::PointF& dpd)
{
    dotsPerDegree = dpd;
}

void RoboCore::Impl::setState(State state)
{
    enum AngleType {Velocity = 0, Position = 1};
    switch (state)
    {
    case State::Search:
        influence.angleType = AngleType::Velocity;
        break;
    case State::Track:
        influence.angleType = AngleType::Position;
        break;
    default:
        break;
    }
}
