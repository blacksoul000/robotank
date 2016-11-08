#ifndef APPLICATION_H
#define APPLICATION_H

#include <QObject>

namespace app
{
    class Application : public QObject
    {
        Q_OBJECT
    public:
        explicit Application();
        ~Application();

        void start();

    private slots:
        void onConnected();

    private:
        class Impl;
        Impl* d;
    };
}

#endif // APPLICATION_H
