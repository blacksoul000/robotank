#include "video_source.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#ifdef PICAM
#include <raspicam/raspicam_cv.h>
#endif

using video::VideoSource;

class VideoSource::Impl
{
public:
    Impl(ros::NodeHandle* nh) : nh(nh), it(*nh) {}
    ros::NodeHandle* nh = nullptr;
    image_transport::ImageTransport it;
    image_transport::Publisher publisher = it.advertise("camera/image", 1);

#ifdef PICAM
    raspicam::RaspiCam_Cv capturer;
#else
    cv::VideoCapture capturer;
#endif
    int fps = 0;
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
#ifdef PICAM
    //set camera params
    d->capturer.set(CV_CAP_PROP_FORMAT, CV_8UC3);

    if(!d->capturer.open())
#else
//    d->capturer.open("/home/user/photos/rae2015/MOV_0002.mp4");
    if(!d->capturer.open(cameraNumber))
#endif
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
#ifdef PICAM
        d->capturer.grab();
        d->capturer.retrieve(frame);
#else
        d->capturer >> frame;
#endif
        if(frame.empty()) continue;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        d->publisher.publish(msg);
        ros::spinOnce();
        cv::waitKey(1);
        loop_rate.sleep();
    }
}
