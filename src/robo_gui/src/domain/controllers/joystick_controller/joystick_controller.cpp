#include "joystick_controller.h"

#include <QSocketNotifier>
#include <QFile>
#include <QDebug>

#ifndef ANDROID
    #include <linux/joystick.h>
#endif //ANDROID

#include <fcntl.h>
#include <unistd.h>

using domain::JoystickController;

class JoystickController::Impl
{
public:
    QString device;
    int fd = -1;
    QSocketNotifier* notifier = nullptr;
};

JoystickController::JoystickController(const QString& device, QObject* parent) :
    QObject(parent),
    d(new Impl)
{
    d->device = device;
}

JoystickController::~JoystickController()
{
    delete d->notifier;
    delete d;
}

void JoystickController::start()
{
    if (d->fd > -1) return;
    d->fd = open(d->device.toUtf8().data(), O_RDONLY | O_NONBLOCK);
    if (d->fd < 0) return;
    d->notifier = new QSocketNotifier(d->fd, QSocketNotifier::Read, this);

    connect(d->notifier, &QSocketNotifier::activated, this, &JoystickController::onReadyRead);
    qDebug() << Q_FUNC_INFO;
}

void JoystickController::stop()
{
    close(d->fd);
    emit finished();
}

void JoystickController::onReadyRead()
{
#ifndef ANDROID
    struct js_event event;
    while (read(d->fd, &event, sizeof(event)) == sizeof(event))
    {
        switch (event.type) {
        case JS_EVENT_AXIS:
            break;
        case JS_EVENT_BUTTON:
            if (event.value)
            {
                emit buttonPressed(event.number);
            }
            else
            {
                emit buttonReleased(event.number);
            }
            break;
        default:
            break;
        }
    }
#endif //ANDROID
}
