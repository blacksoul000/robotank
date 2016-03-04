#ifndef ARDUINO_EXCHANGER_H
#define ARDUINO_EXCHANGER_H

#include <string>

namespace ros
{
    class NodeHandle;
}

namespace arduino
{
    class ArduinoExchanger
    {
    public:
        ArduinoExchanger(ros::NodeHandle* nh, const std::string& path, int address);
        ~ArduinoExchanger();

        void process();

    private:
        class Impl;
        Impl* d;
    };
} // namespace arduino

#endif //ARDUINO_EXCHANGER_H
