#include "video_source/video_source.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_source");
    ros::NodeHandle nh;

    video::VideoSource source(&nh, 25);
    source.start(0);
    return 0;
}
