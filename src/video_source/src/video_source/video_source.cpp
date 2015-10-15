#include "video_source.h"


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QTimer>
#include <QDebug>

using video::VideoSource;

namespace
{
    const int msInSec = 1000;
}

class VideoSource::Impl
{
public:
    Impl(ros::NodeHandle* nh) : it(*nh) {}
    image_transport::ImageTransport it;
    image_transport::Publisher publisher = it.advertise("camera/image", 1);

    cv::VideoCapture capturer;
    QTimer* timer = nullptr;
};

VideoSource::VideoSource(ros::NodeHandle* nh, int fps, QObject* parent) :
    QObject(parent),
    d(new Impl(nh))
{
    d->timer = new QTimer(this);
    d->timer->setInterval(::msInSec / fps);
    connect(d->timer, &QTimer::timeout, this, &VideoSource::capture);
}

VideoSource::~VideoSource()
{
    this->stop();
    delete d;
}

bool VideoSource::start(int cameraNumber)
{
    d->capturer.open(cameraNumber); //camera number
    if(!d->capturer.isOpened())
    {
        qWarning() << "Failed to open camera";
        return false;
    }
    d->timer->start();
    return true;
}

void VideoSource::stop()
{
    d->timer->stop();
}

void VideoSource::capture()
{
    cv::Mat frame;
    d->capturer >> frame;
    if(frame.empty()) return;

    sensor_msgs::ImagePtr msg;

    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    d->publisher.publish(msg);
    cv::waitKey(1);
    ros::spinOnce();
}
