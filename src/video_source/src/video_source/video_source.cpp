#include "video_source.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using video::VideoSource;

class VideoSource::Impl
{
public:
    Impl(ros::NodeHandle* nh) : nh(nh), it(*nh) {}
    ros::NodeHandle* nh = nullptr;
    image_transport::ImageTransport it;
    image_transport::Publisher publisher = it.advertise("camera/image", 1);

    cv::VideoCapture capturer;
    int fps = 1;
    volatile bool stop = false;
};

VideoSource::VideoSource(ros::NodeHandle* nh, int fps) :
    d(new Impl(nh))
{
    d->fps = fps;
}

VideoSource::~VideoSource()
{
    this->stop();
    delete d;
}

void VideoSource::start(int cameraNumber)
{
//     d->capturer.open(cameraNumber); //camera number
    d->capturer.open("/home/user/photos/rae2015/MOV_0002.mp4"); //camera number
    if(!d->capturer.isOpened())
    {
        ROS_WARN("Failed to open camera");
        return;
    }
    this->capture();
}

void VideoSource::stop()
{
    d->stop = true;
}

void VideoSource::capture()
{
    ros::Rate loop_rate(d->fps);
    sensor_msgs::ImagePtr msg;
    while (d->nh->ok() && !d->stop) 
    {
        cv::Mat frame;
        d->capturer >> frame;
        if(frame.empty()) continue;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        d->publisher.publish(msg);
        ros::spinOnce();
        cv::waitKey(1);
        loop_rate.sleep();
    }
}
