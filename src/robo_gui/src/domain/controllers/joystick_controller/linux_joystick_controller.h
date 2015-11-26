#ifndef LINUX_JOYSTICK_CONTROLLER_H
#define LINUX_JOYSTICK_CONTROLLER_H

#include "abstract_joystick_controller.h"

namespace domain
{
    class LinuxJoystickController : public AbstractJoystickController
    {
        Q_OBJECT
    public:
        LinuxJoystickController(const QString& device, QObject* parent = nullptr);
        ~LinuxJoystickController() override;

        void start() override;
        void stop() override;

    private slots:
        void onReadyRead();

    private:
        class Impl;
        Impl* d;
    };

} //namespace domain

#endif //LINUX_JOYSTICK_CONTROLLER_H
