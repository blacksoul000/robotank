#include "tracker.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using va::Tracker;

class Tracker::Impl
{
public:
    Impl(ros::NodeHandle* nh) : it(*nh) {}
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSubscriber = it.subscribe("camera/image", 1,
                    boost::bind(&Tracker::Impl::onNewFrame, this, _1));

//    Rect target;

    void onNewFrame(const sensor_msgs::ImageConstPtr& msg);
};

Tracker::Tracker(ros::NodeHandle* nh) :
    d(new Impl(nh))
{
}

Tracker::~Tracker()
{
    delete d;
}

void Tracker::Impl::onNewFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat frame =  cv_bridge::toCvShare(msg, "bgr8")->image;
//    ROS_WARN("Encoding \"%s\" )", ptr->encoding.c_str());
    ROS_INFO("caught image");
}
