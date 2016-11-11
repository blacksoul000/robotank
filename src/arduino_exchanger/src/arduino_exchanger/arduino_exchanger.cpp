#include "arduino_exchanger.h"

//msgs
#include "robo_core/Influence.h"
#include "robo_core/PointF.h"
#include "robo_core/Point3D.h"
#include <std_msgs/Empty.h>

//ros
#include <ros/ros.h>

//linux
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>

using arduino::ArduinoExchanger;
using Clock = std::chrono::high_resolution_clock;

namespace
{
#pragma pack(push, 1)
    struct ArduinoPkg
    {
        int16_t gunH = 0;
        int16_t gunV = 0;
        int16_t cameraV = 0;
        int16_t yaw = 0;
        int16_t pitch = 0;
        int16_t roll = 0;
    };
    struct RaspberryPkg
    {
        int8_t bySpeed : 1;
        int8_t deviceId : 4;
        int8_t reserve : 3;
        int16_t x = 0;
        int16_t y = 0;
    };
#pragma pack(pop)

    constexpr double positionCoef = 360.0 / 32767;
    constexpr double influenceCoef = 90.0 / 32767;
    const int timeout = 500; // ms
} // namespace

class ArduinoExchanger::Impl
{
public:
    std::string device;
    int fd = -1;
    std::vector< char > buffer;
    std::vector< char > prefix = { 0x55, 0x55 };
    bool waitPrefix = true;
    Clock::time_point lastTime;
    ArduinoPkg offsets;
    ArduinoPkg lastData;

    std::list< ros::Subscriber > subscribers;

    ros::Publisher gunPositionPub;
    ros::Publisher cameraPositionPub;
    ros::Publisher yprPub;
    ros::Subscriber influenceSub;

    bool open();
    bool isOpened() const;
    void readData();
    void setArduinoOnline(bool set);

    void onInfluence(const robo_core::Influence& influence);
    void onGunCalibrate(std_msgs::Empty);
    void onCameraCalibrate(std_msgs::Empty);
    void onGyroCalibrate(std_msgs::Empty);

    int setInterfaceAttribs (int fd, int speed, int parity);
    void setBlocking (int fd, int should_block);

private:
    bool arduinoOnline = true;
};

ArduinoExchanger::ArduinoExchanger(ros::NodeHandle* nh, const std::string& path):
    d(new Impl)
{
    d->device = path;
    d->gunPositionPub = nh->advertise< robo_core::PointF >("gun/position", 1);
    d->cameraPositionPub = nh->advertise< robo_core::PointF >("camera/position", 1);
    d->yprPub = nh->advertise< robo_core::Point3D >("robo/ypr", 1);
    d->subscribers.push_back(nh->subscribe("core/influence", 10, &ArduinoExchanger::Impl::onInfluence, d));
    d->subscribers.push_back(nh->subscribe("gun/calibrate", 1, &ArduinoExchanger::Impl::onGunCalibrate, d));
    d->subscribers.push_back(nh->subscribe("camera/calibrate", 1, &ArduinoExchanger::Impl::onCameraCalibrate, d));
    d->subscribers.push_back(nh->subscribe("ypr/calibrate", 1, &ArduinoExchanger::Impl::onGyroCalibrate, d));

    d->lastTime = Clock::now();
}

ArduinoExchanger::~ArduinoExchanger()
{
    close(d->fd);
    delete d;
}

void ArduinoExchanger::process()
{
    if (!d->isOpened() && !d->open()) return;

    d->readData();
    if (std::chrono::duration<double, std::milli>(Clock::now() - d->lastTime).count() > ::timeout) 
    {
        d->setArduinoOnline(false);
    }
}

void ArduinoExchanger::Impl::readData()
{
    char c;
    bool hasNewData = false;
    while (read(fd, &c, 1) > 0)
    {
        buffer.push_back(c);
        hasNewData = true;
    }

    if (!hasNewData) return;

    for (char cc: buffer)
        std::cout << std::hex << (unsigned short)cc << " ";
    std::cout << std::endl;

    if (waitPrefix)
    {
        if (buffer.size() < prefix.size()) return;
        auto found = std::search(begin(buffer), end(buffer), begin(prefix), end(prefix));
        if (found == end(buffer))
        {
            buffer.erase(buffer.begin(), found - prefix.size());
//            std::cout << "WP: Not found. new size = " << buffer.size() << std::endl;
            return;
        }
        else
        {
//            std::cout << "WP:: Found at: " << std::distance(buffer.begin(), found) << std::endl;
            buffer.erase(buffer.begin(), found);
            waitPrefix = false;
        }
    }

    if (!waitPrefix)
    {
        const int packetSize = prefix.size() + sizeof(::ArduinoPkg);
        if (buffer.size() < packetSize) return;

        ::ArduinoPkg pkg = *reinterpret_cast<::ArduinoPkg *>(&buffer[prefix.size()]);
        buffer.erase(buffer.begin(), buffer.begin() + packetSize);
        waitPrefix = true;

        robo_core::PointF gun;
        gun.x = (pkg.gunH - offsets.gunH) * ::positionCoef;
        gun.y = (pkg.gunV - offsets.gunV) * ::positionCoef;
        gunPositionPub.publish(gun);

        robo_core::PointF camera;
        camera.x = 0;
        camera.y = (pkg.cameraV - offsets.cameraV) * ::positionCoef;
        cameraPositionPub.publish(camera);

        robo_core::Point3D ypr;
        ypr.x = (pkg.yaw - offsets.yaw) * ::positionCoef;
        ypr.y = (pkg.pitch - offsets.pitch) * ::positionCoef;
        ypr.z = (pkg.roll - offsets.roll) * ::positionCoef;
        yprPub.publish(ypr);

        memcpy(&lastData, &pkg, sizeof(ArduinoPkg));

        ROS_WARN("gun: %f, %f, %d, %d", gun.x, gun.y, pkg.gunH, pkg.gunV);
        ROS_WARN("camera: %f, %d", camera.y, pkg.cameraV);
        ROS_WARN("ypr: %f, %f, %f, %d, %d, %d", ypr.x, ypr.y, ypr.z, pkg.yaw, pkg.pitch, pkg.roll);

        lastTime = Clock::now();
        setArduinoOnline(true);
    }
}

//------------------------------------------------------------------------------------
bool ArduinoExchanger::Impl::open()
{
    fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);
    if (fd < 0)
    {
        ROS_DEBUG_ONCE("Failed to open %s(%s)", device.c_str(), strerror (errno));
        return false;
    }

    fcntl(fd, F_SETFL, FNDELAY);	//sets no delay when reading if no dt available
    this->setInterfaceAttribs(fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    this->setBlocking(fd, 0);                // set no blocking
    return true;
}

bool ArduinoExchanger::Impl::isOpened() const
{
    return fd >= 0;
}

int ArduinoExchanger::Impl::setInterfaceAttribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        ROS_WARN("Error %d from tcgetattr.", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        ROS_WARN("Error %d from tcsetattr.", errno);
        return -1;
    }
    return 0;
}

void ArduinoExchanger::Impl::setBlocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        ROS_WARN("Error %d from tggetattr.", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        ROS_WARN("Error %d setting term attributes.", errno);
    }
}

void ArduinoExchanger::Impl::onInfluence(const robo_core::Influence& influence)
{
    ROS_WARN("onInfluence: %d, %d, %d", influence.deviceId, influence.x, influence.y);
    ::RaspberryPkg pkg;
    pkg.bySpeed = influence.bySpeed;
    pkg.deviceId = influence.deviceId;
    pkg.x = influence.x / ::influenceCoef;
    pkg.y = influence.y / ::influenceCoef;

    std::string str(prefix.begin(), prefix.end());
    write(fd, str.data(), str.length());
    if (write(fd, reinterpret_cast<const char *>(&pkg), sizeof(pkg)) != sizeof(pkg))
    {
        ROS_WARN("Failed to write to the bus.");
    }
}

void ArduinoExchanger::Impl::onGunCalibrate(std_msgs::Empty)
{
    ROS_WARN("Calibrate gun");
    offsets.gunH = lastData.gunH;
    offsets.gunV = lastData.gunV;
}

void ArduinoExchanger::Impl::onCameraCalibrate(std_msgs::Empty)
{
    ROS_WARN("Calibrate camera");
    offsets.cameraV = lastData.cameraV;
}

void ArduinoExchanger::Impl::onGyroCalibrate(std_msgs::Empty)
{
    ROS_WARN("Calibrate gyro");
    offsets.yaw = lastData.yaw;
    offsets.pitch = lastData.pitch;
    offsets.roll = lastData.roll;
}

void ArduinoExchanger::Impl::setArduinoOnline(bool online)
{
    if (online == arduinoOnline) return;
    if (online)
    {
        ROS_WARN("Arduino online");
    }
    else
    {
        ROS_WARN("Arduino offline");
    }
    arduinoOnline = online;
}
