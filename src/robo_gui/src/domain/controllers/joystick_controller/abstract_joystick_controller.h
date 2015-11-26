#ifndef ABSTRACT_JOYSTICK_CONTROLLER_H
#define ABSTRACT_JOYSTICK_CONTROLLER_H

#include <QObject>

namespace domain
{
    class AbstractJoystickController : public QObject
    {
        Q_OBJECT
    public:
        AbstractJoystickController(QObject* parent = nullptr);
        virtual ~AbstractJoystickController();

        virtual void start() = 0;
        virtual void stop() = 0;

    signals:
//        void valueChanged()
        void buttonPressed(int button);
        void buttonReleased(int button);

        void finished();
    };

} //namespace domain

#endif //ABSTRACT_JOYSTICK_CONTROLLER_H
