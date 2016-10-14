#include "arduino_exchanger.h"

//msgs
#include "robo_core/Influence.h"
#include "robo_core/PointF.h"

//ros
#include <ros/ros.h>

//linux
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

using arduino::ArduinoExchanger;

namespace
{
#pragma pack(push, 1)
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
#pragma pack(pop)

    constexpr double positionCoef = 360.0 / 32767;
    constexpr double influenceCoef = 90.0 / 32767;
} // namespace

class ArduinoExchanger::Impl
{
public:
    std::string device;
    int fd = -1;
    std::vector< char > buffer;
    std::vector< char > prefix = { 0x55, 0x55 };
    bool waitPrefix = true;

    ros::Publisher positionPub;
    ros::Subscriber influenceSub;

    bool open();
    bool isOpened() const;
    void readData();
    void onInfluence(const robo_core::Influence& influence);

    int setInterfaceAttribs (int fd, int speed, int parity);
    void setBlocking (int fd, int should_block);
};

ArduinoExchanger::ArduinoExchanger(ros::NodeHandle* nh, const std::string& path):
    d(new Impl)
{
    d->device = path;
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

    d->readData();
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

//    for (char cc: buffer)
//        std::cout << std::hex << (unsigned short)cc << " ";
//    std::cout << std::endl;

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

        robo_core::PointF msg;
        msg.x = pkg.x * ::positionCoef;
        msg.y = pkg.y * ::positionCoef;
        positionPub.publish(msg);
        ROS_WARN("position: %d, %d, %f, %f", pkg.x, pkg.y, msg.x, msg.y);
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
        ROS_WARN("Failed to write to the i2c bus.");
    }
}
