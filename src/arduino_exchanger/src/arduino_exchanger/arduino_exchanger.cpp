#include "arduino_exchanger.h"

//msgs
#include "robo_core/Influence.h"
#include "robo_core/PointF.h"

//ros
#include <ros/ros.h>

//linux
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using arduino::ArduinoExchanger;

namespace
{
    struct ArduinoPkg
    {
        int16_t x = 0;
        int16_t y = 0;
    };
    struct RaspberryPkg
    {
        int8_t bySpeed : 1;
        int8_t deviceId : 4;
        int8_t reserve : 3;
        int16_t x = 0;
        int16_t y = 0;
    };

    constexpr double positionCoef = 360.0 / 32767;
    constexpr double influenceCoef = 90.0 / 32767;
} // namespace

class ArduinoExchanger::Impl
{
public:
    std::string device;
    int address = 0;
    int fd = -1;

    ros::Publisher positionPub;
    ros::Subscriber influenceSub;

    bool open();
    bool isOpened() const;
    void onInfluence(const robo_core::Influence& influence);
};

ArduinoExchanger::ArduinoExchanger(ros::NodeHandle* nh, const std::string& path, int address):
    d(new Impl)
{
    d->device = path;
    d->address = address;
    d->positionPub = nh->advertise< robo_core::PointF >("camera/position", 1);
    d->influenceSub = nh->subscribe("core/influence", 10, &ArduinoExchanger::Impl::onInfluence, d);
}

ArduinoExchanger::~ArduinoExchanger()
{
    close(d->fd);
    delete d;
}

void ArduinoExchanger::process()
{
    if (!d->isOpened() && !d->open()) return;

    ::ArduinoPkg pkg;
    if (read(d->fd, reinterpret_cast<char *>(&pkg), sizeof(pkg)) != sizeof(pkg))
    {
        ROS_WARN("Failed to read from the i2c bus.");
        return;
    }
    robo_core::PointF msg;
    msg.x = pkg.x * ::positionCoef;
    msg.y = pkg.y * ::positionCoef;
    d->positionPub.publish(msg);
}

//------------------------------------------------------------------------------------
bool ArduinoExchanger::Impl::open()
{
    fd = ::open(device.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0)
    {
        ROS_DEBUG_ONCE("Failed to open %s", device.c_str());
        return false;
    }
    if (ioctl(fd, I2C_SLAVE, address) < 0)
    {
        ROS_DEBUG_ONCE("Failed to acquire bus access and/or talk to slave.");
        return false;
    }
    return true;
}

bool ArduinoExchanger::Impl::isOpened() const
{
    return fd >= 0;
}

void ArduinoExchanger::Impl::onInfluence(const robo_core::Influence& influence)
{
    ROS_WARN("onInfluence: %d, %f, %f", influence.deviceId, influence.x, influence.y);
    ::RaspberryPkg pkg;
    pkg.bySpeed = influence.bySpeed;
    pkg.deviceId = influence.deviceId;
    pkg.x = influence.x / ::influenceCoef;
    pkg.y = influence.y / ::influenceCoef;
    if (write(fd, reinterpret_cast<const char *>(&pkg), sizeof(pkg)) != sizeof(pkg))
    {
        ROS_WARN("Failed to write to the i2c bus.");
    }
}
