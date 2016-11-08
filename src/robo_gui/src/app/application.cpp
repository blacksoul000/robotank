#include "application.h"
#include "main_window.h"
#include "master_status_checker.h"

#include <ros/ros.h>

#include <QCoreApplication>
#include <QNetworkInterface>
#include <QScopedPointer>
#include <QHostAddress>
#include <QSettings>
#include <QDebug>

using app::Application;

namespace
{
    const QString settingsFileName = "robotank.cfg";
}

class Application::Impl
{
public:
    QScopedPointer< QSettings > settings;
    QScopedPointer< app::MasterStatusChecker > checker;
    robo::MainWindow* robo = nullptr;
    ros::AsyncSpinner* spinner = nullptr;
    ros::NodeHandle* nh = nullptr;
    bool started = false;

    int argc = 3;
    char** argv = nullptr;
};

Application::Application() :
    QObject(nullptr),
    d(new Impl)
{
    d->settings.reset(new QSettings(::settingsFileName, QSettings::NativeFormat));
    d->checker.reset(new app::MasterStatusChecker(d->settings.data()));

    connect(d->checker.data(), &app::MasterStatusChecker::connected,
            this, &Application::onConnected);
}

Application::~Application()
{
    if (d->started)
    {
        d->spinner->stop();
        delete d->spinner;
        delete d->nh;
        delete d->robo;
    }

    d->settings->sync();
    delete d;
}

void Application::start()
{
    d->checker->start();
}

void Application::onConnected()
{
    // This is need to correct run on Android
    QHostAddress selfIp;
    foreach (const auto& address, QNetworkInterface::allAddresses())
    {
        if (address.protocol() == QAbstractSocket::IPv4Protocol
                && address != QHostAddress::LocalHost)
             selfIp = address;
    }

    std::string appNameString = QCoreApplication::applicationFilePath().toStdString();
    std::string selfIpString = "__ip:=" + selfIp.toString().toStdString();
    std::string masterIpString = "__master:=http://" + d->checker->ip().toStdString() + ":11311";
    d->checker.reset();

    char s0[appNameString.length() + 1];
    char s1[masterIpString.length() + 1];
    char s2[selfIpString.length() + 1];
    strcpy(s0, appNameString.c_str());
    strcpy(s1, masterIpString.c_str());
    strcpy(s2, selfIpString.c_str());
    char* argv[] = {s0, s1, s2};
    d->argv = argv;
    // android stuff end

    ros::init(d->argc, d->argv, "robo_gui_node");
    d->nh = new ros::NodeHandle();
    d->spinner = new ros::AsyncSpinner(1);  // Use 1 thread
    d->robo = new robo::MainWindow(d->nh, d->settings.data());
    d->robo->loadSettings();
    d->spinner->start();

    d->started = true;
}
