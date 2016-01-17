#include "video_source.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/UInt8.h"

#ifdef PICAM
#include <raspicam/raspicam_cv.h>
#endif

using video::VideoSource;

namespace
{
    const int defaultQuality = 15;
    const int width = 640;
    const int height = 480;
} //namespace

class VideoSource::Impl
{
public:
    Impl(ros::NodeHandle* nh) : nh(nh), it(*nh)
    {
        imageQualitySub = nh->subscribe("camera/image/quality", 0,
                    &VideoSource::Impl::onQualityChangeRequest, this);
    }
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
    ros::Subscriber imageQualitySub;

    void onQualityChangeRequest(const std_msgs::UInt8& msg);
    void onQualityChangeRequestImpl(int quality);
};

void VideoSource::Impl::onQualityChangeRequest(const std_msgs::UInt8& msg)
{
    this->onQualityChangeRequestImpl(msg.data);
}

void VideoSource::Impl::onQualityChangeRequestImpl(int quality)
{
//    rosrun dynamic_reconfigure dynparam set -t1 /camera/image/compressed jpeg_quality 15
    std::string s = "rosrun dynamic_reconfigure dynparam set -t1 "
                    "/camera/image/compressed jpeg_quality "
            + std::to_string(quality);
    popen(s.c_str(), "r");
    // system(s.c_str()); does not work by unknown reason
}

//----------------------------------------------------------------------------
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
    // set camera params
    // 2592x1944 - native resolution
    d->capturer.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    d->capturer.set(CV_CAP_PROP_FRAME_HEIGHT, ::height);
    d->capturer.set(CV_CAP_PROP_FRAME_WIDTH, ::width);
    d->capturer.set(CV_CAP_PROP_BRIGHTNESS, 85);
    d->capturer.set(CV_CAP_PROP_CONTRAST, 95);

    if(!d->capturer.open())
#else
    d->capturer.set(CV_CAP_PROP_FRAME_WIDTH, ::width);
    d->capturer.set(CV_CAP_PROP_FRAME_HEIGHT, ::height);
    if(!d->capturer.open(cameraNumber))
#endif
    {
        ROS_WARN("Failed to open camera");
        return;
    }
    int quality = 0;
    ros::param::param< int >("camera/image/quality", quality, ::defaultQuality);
    d->onQualityChangeRequestImpl(quality);
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
//        frame = cv::imread("/home/blacksoul/workspace/robotank/1024x768.jpg", CV_LOAD_IMAGE_COLOR);
#endif
        if(frame.empty()) continue;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        d->publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
