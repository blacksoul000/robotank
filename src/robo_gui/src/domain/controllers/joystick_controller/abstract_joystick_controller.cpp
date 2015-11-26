#include "abstract_joystick_controller.h"

#include <QDebug>

using domain::AbstractJoystickController;

AbstractJoystickController::AbstractJoystickController(QObject* parent) :
    QObject(parent)
{
}

AbstractJoystickController::~AbstractJoystickController()
{
}
