#ifndef JOYSTICK_MODEL_H
#define JOYSTICK_MODEL_H

#include <QObject>

namespace domain
{
    class JoystickModel : public QObject
    {
        Q_OBJECT
    public:
        JoystickModel(QObject* parent = nullptr);
        ~JoystickModel();

    public slots:
        void onButtonPressed(int button);
        void onButtonReleased(int button);

    private:
        class Impl;
        Impl* d;
    };
}

#endif //JOYSTICK_MODEL_H
