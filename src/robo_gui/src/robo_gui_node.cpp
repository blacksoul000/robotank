#include "main_window.h"

#include <ros/ros.h>

#include <QGuiApplication>
#include <QNetworkInterface>
#include <QHostAddress>

int main(int argc, char** argv)
{
    QGuiApplication app(argc, argv);
    app.setApplicationName("Robotank");

#ifdef ANDROID
// Some magic for android run
// FIXME - don't hardcode ips!

    QHostAddress selfIp;
    foreach (const auto& address, QNetworkInterface::allAddresses())
    {
        if (address.protocol() == QAbstractSocket::IPv4Protocol
                && address != QHostAddress::LocalHost)
             selfIp = address;
    }
    std::string ip = "__ip:=" + selfIp.toString().toStdString();
    char s0[] = "robo_gui_droid";
    char s1[] = "__master:=http://192.168.1.3:11311";
    char s2[ip.length() + 1];
    strcpy(s2, ip.c_str());
    char* v[] = {s0, s1, s2};
    int c = 3;

    ros::init(c, &v[0], "robo_gui_droid");
#else
    ros::init(argc, argv, "robo_gui");
#endif
    ros::NodeHandle nh;

    robo::MainWindow robo(&nh);
    robo.loadSettings();
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    bool res = app.exec();
    spinner.stop();
    return res;
}
