#include "main_window.h"

#include <ros/ros.h>

#include <QGuiApplication>

int main(int argc, char** argv)
{
    QGuiApplication app(argc, argv);
    app.setApplicationName("Robotank");

#ifdef ANDROID
// Some magic for android run
// FIXME - don't hardcode ips!
    int c = 3;
    char *v[] = {"robo_gui_droid" , "__master:=http://192.168.1.213:11311", "__ip:=192.168.1.80"};

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
