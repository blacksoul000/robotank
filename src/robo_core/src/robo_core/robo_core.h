#ifndef ROBO_CORE_H
#define ROBO_CORE_H

#include <string>

namespace ros
{
    class NodeHandle;
}

namespace robo_core
{
    class RoboCore
    {
    public:
        enum class State
        {
            Search,
            Track
        };

        RoboCore(ros::NodeHandle* nh);
        ~RoboCore();

    private:
        class Impl;
        Impl* d;
    };
} // namespace robo_core

#endif //ROBO_CORE_H
