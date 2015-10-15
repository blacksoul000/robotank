#include "video_source/video_source.h"

#include <ros/ros.h>

#include <QCoreApplication>

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    ros::init(argc, argv, "video_source");
    ros::NodeHandle nh;

    video::VideoSource source(&nh, 30);
    if (!source.start(0)) return 1;

    return app.exec();
}
