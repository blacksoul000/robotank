#ifndef GAMEPAD_CONTROLLER_H
#define GAMEPAD_CONTROLLER_H

#include <string>

namespace ros
{
    class NodeHandle;
}

namespace gamepad
{
    class GamepadController
    {
    public:
        GamepadController(ros::NodeHandle* nh, const std::string& path);
        ~GamepadController();

        void process();

    private:
        class Impl;
        Impl* d;
    };
} // namespace gamepad

#endif //GAMEPAD_CONTROLLER_H
