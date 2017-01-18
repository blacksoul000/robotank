#ifndef GPIO_CONTROLLER_H
#define GPIO_CONTROLLER_H

namespace ros
{
    class NodeHandle;
}

namespace gpio
{
    class Controller
    {
    public:
        Controller(ros::NodeHandle* nh);
        ~Controller();

        void process();

    private:
        class Impl;
        Impl* d;
    };
} // namespace gpio

#endif //GPIO_CONTROLLER_H
