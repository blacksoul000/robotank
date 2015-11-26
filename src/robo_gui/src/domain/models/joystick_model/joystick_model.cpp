#include "joystick_model.h"
#include "android_joystick_controller.h"
#include "linux_joystick_controller.h"

#include <QThread>
#include <QDebug>

using domain::JoystickModel;
using domain::AbstractJoystickController;

class JoystickModel::Impl
{
public:
    AbstractJoystickController* controller = nullptr;
    QThread worker;
};

JoystickModel::JoystickModel(QObject* parent) :
    QObject(parent),
    d(new Impl)
{
    //TODO - factory
#ifndef ANDROID
    d->controller = new LinuxJoystickController("/dev/input/js0");
#else
    d->controller = new AndroidJoystickController();
#endif

    d->controller->moveToThread(&d->worker);

    connect(&d->worker, &QThread::started, d->controller, &AbstractJoystickController::start);
    connect(d->controller, &AbstractJoystickController::finished, &d->worker, &QThread::quit);

    connect(d->controller, &AbstractJoystickController::buttonPressed,
            this, &JoystickModel::onButtonPressed);
    connect(d->controller, &AbstractJoystickController::buttonReleased,
            this, &JoystickModel::onButtonReleased);

    d->worker.start();
}

JoystickModel::~JoystickModel()
{
    d->controller->stop();
    d->worker.wait();
    delete d->controller;
    delete d;
}

void JoystickModel::onButtonPressed(int button)
{
    qDebug() << Q_FUNC_INFO << button;
}

void JoystickModel::onButtonReleased(int button)
{
    qDebug() << Q_FUNC_INFO << button;
}

