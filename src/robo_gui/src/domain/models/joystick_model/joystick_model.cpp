#include "joystick_model.h"
#include "joystick_controller.h"

#include <QThread>
#include <QDebug>

using domain::JoystickModel;
using domain::JoystickController;

class JoystickModel::Impl
{
public:
    JoystickController* controller = nullptr;
    QThread worker;
};

JoystickModel::JoystickModel(QObject* parent) :
    QObject(parent),
    d(new Impl)
{
    d->controller = new JoystickController("/dev/input/js0");
    d->controller->moveToThread(&d->worker);

    connect(&d->worker, &QThread::started, d->controller, &JoystickController::start);
    connect(d->controller, &JoystickController::finished, &d->worker, &QThread::quit);

    connect(d->controller, &JoystickController::buttonPressed,
            this, &JoystickModel::onButtonPressed);
    connect(d->controller, &JoystickController::buttonReleased,
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

