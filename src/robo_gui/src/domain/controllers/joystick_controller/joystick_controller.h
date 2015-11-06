#ifndef JOYSTICK_CONTROLLER_H
#define JOYSTICK_CONTROLLER_H

#include <QObject>

namespace domain
{
    class JoystickController : public QObject
    {
        Q_OBJECT
    public:
        JoystickController(const QString& device, QObject* parent = nullptr);
        ~JoystickController();

    void start();
    void stop();

    public slots:
        void onReadyRead();

    signals:
//        void valueChanged()
        void buttonPressed(int button);
        void buttonReleased(int button);

        void finished();

    private:
        class Impl;
        Impl* d;
    };

} //namespace domain

#endif //JOYSTICK_CONTROLLER_H
