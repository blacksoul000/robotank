#include "main_window.h"

#include <ros/ros.h>

#include <QGuiApplication>

int main(int argc, char** argv)
{
    QGuiApplication app(argc, argv);
    app.setApplicationName("Robotank");

//    ros::init(argc, argv, "robo_gui");
//    ros::NodeHandle nh;

//    robo::MainWindow robo(&nh);
    robo::MainWindow robo(nullptr);
//    robo.loadSettings();
//    ros::AsyncSpinner spinner(1); // Use 1 thread
//    spinner.start();
    bool res = app.exec();
//    spinner.stop();
    return res;
}
