#include "android_joystick_controller.h"

#include <QCoreApplication>
#include <QDebug>

using domain::AndroidJoystickController;

class AndroidJoystickController::Impl
{
public:
};

AndroidJoystickController::AndroidJoystickController(QObject* parent) :
    AbstractJoystickController(parent),
    QAbstractNativeEventFilter(),
    d(new Impl)
{
//    qApp->installNativeEventFilter(this);
}

AndroidJoystickController::~AndroidJoystickController()
{
    delete d;
}

void AndroidJoystickController::start()
{
}

void AndroidJoystickController::stop()
{
    emit finished();
}

bool AndroidJoystickController::nativeEventFilter(const QByteArray& eventType, void* message, long*)
{
//    qDebug() << eventType;
////    if (eventType == "xcb_generic_event_t") {
////        return app->xcbEventFilter(message);
////    }
//    return false;
}

//float x = AMotionEvent_getX(event, 0);
//float y = AMotionEvent_getY(event, 0);
//if (_getAxisValue) {
//    // take the hat switches into account too, so that either the
//    // regular axes or the hat axes can be used to navigate UIs
//    x += _getAxisValue(event, AXIS_HAT_X, 0);
//    y += _getAxisValue(event, AXIS_HAT_Y, 0);
//    x = Clamp(x, -1.0f, 1.0f);
//    y = Clamp(y, -1.0f, 1.0f);
//}
