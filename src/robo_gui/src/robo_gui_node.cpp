#include "application.h"

#include <QGuiApplication>

int main(int argc, char** argv)
{
    QGuiApplication app(argc, argv);
    app.setApplicationName("Robotank");

    app::Application application;
    application.start();

    return app.exec();
}
