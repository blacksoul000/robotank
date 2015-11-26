#include "linux_joystick_controller.h"

#include <QSocketNotifier>
#include <QFile>
#include <QDebug>

#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>

using domain::LinuxJoystickController;

class LinuxJoystickController::Impl
{
public:
    QString device;
    int fd = -1;
    QSocketNotifier* notifier = nullptr;
};

LinuxJoystickController::LinuxJoystickController(const QString& device, QObject* parent) :
    AbstractJoystickController(parent),
    d(new Impl)
{
    d->device = device;
}

LinuxJoystickController::~LinuxJoystickController()
{
    delete d->notifier;
    delete d;
}

void LinuxJoystickController::start()
{
    if (d->fd > -1) return;
    d->fd = open(d->device.toUtf8().data(), O_RDONLY | O_NONBLOCK);
    if (d->fd < 0) return;
    d->notifier = new QSocketNotifier(d->fd, QSocketNotifier::Read, this);

    connect(d->notifier, &QSocketNotifier::activated, this, &LinuxJoystickController::onReadyRead);
}

void LinuxJoystickController::stop()
{
    close(d->fd);
    emit finished();
}

void LinuxJoystickController::onReadyRead()
{
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
}
