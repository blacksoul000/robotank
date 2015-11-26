#ifndef ANDROID_JOYSTICK_CONTROLLER_H
#define ANDROID_JOYSTICK_CONTROLLER_H

#include "abstract_joystick_controller.h"

#include <QAbstractNativeEventFilter>

namespace domain
{
    class AndroidJoystickController : public AbstractJoystickController, public QAbstractNativeEventFilter
    {
        Q_OBJECT
    public:
        AndroidJoystickController(QObject* parent = nullptr);
        ~AndroidJoystickController() override;

        void start() override;
        void stop() override;

    protected:
        bool nativeEventFilter(const QByteArray& eventType, void* message, long*) override;

    private:
        class Impl;
        Impl* d;
    };

} //namespace domain

#endif //ANDROID_JOYSTICK_CONTROLLER_H
