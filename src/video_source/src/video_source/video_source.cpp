#include "video_source.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//msgs
#include "std_msgs/UInt8.h"
#include "video_source/PointF.h"

#ifdef PICAM
#include <raspicam/raspicam_cv.h>
#endif

using video::VideoSource;

namespace
{
    const int defaultQuality = 15;
    const int width = 640;
    const int height = 480;

#ifdef PICAM
    const double fieldOfViewH = 53.5; // +/- 0.13 degrees
    const double fieldOfViewV = 41.41; // +/- 0.11 degress
#else
    // set proper values for used camera
    const double fieldOfViewH = 53.5; // +/- 0.13 degrees
    const double fieldOfViewV = 41.41; // +/- 0.11 degress
#endif // PICAM
} //namespace

class VideoSource::Impl
{
public:
//    Impl(ros::NodeHandle* nh) : nh(nh), it(*nh)
//    {
//        imageQualitySub = nh->subscribe("camera/image/quality", 0,
//                    &VideoSource::Impl::onQualityChangeRequest, this);
//    }
//    ros::NodeHandle* nh = nullptr;
//    image_transport::ImageTransport it;
//    image_transport::Publisher publisher = it.advertise("camera/image", 1);
    image_transport::Publisher imagePublisher;
#ifdef PICAM
    raspicam::RaspiCam_Cv capturer;
#else
    cv::VideoCapture capturer;
#endif //PICAM
    int fps = 0;
    volatile bool stop = false;
//    ros::Subscriber imageQualitySub;
    image_transport::Publisher publisher;
    ros::Publisher dotsPerDegreePublisher;

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
    FILE* f = popen(s.c_str(), "r");
    pclose(f);
    // system(s.c_str()); does not work by unknown reason
}

//----------------------------------------------------------------------------
VideoSource::VideoSource(ros::NodeHandle* nh, int fps) :
    d(new Impl)
{
    d->fps = fps;

    image_transport::ImageTransport it(*nh);
    d->imagePublisher = it.advertise("camera/image", 1);
    d->dotsPerDegreePublisher = nh->advertise< video_source::PointF >("camera/dotsPerDegree", 1);
    nh->subscribe("camera/image/quality", 0, &VideoSource::Impl::onQualityChangeRequest, d);
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
    d->capturer.set(CV_CAP_PROP_BRIGHTNESS, 75);
    d->capturer.set(CV_CAP_PROP_CONTRAST, 95);

    if(!d->capturer.open())
#else
    d->capturer.set(CV_CAP_PROP_FRAME_WIDTH, ::width);
    d->capturer.set(CV_CAP_PROP_FRAME_HEIGHT, ::height);
    if(!d->capturer.open(cameraNumber))
#endif //PICAM
    {
        ROS_WARN("Failed to open camera");
        return;
    }
    int quality = 0;
    ros::param::param< int >("camera/image/quality", quality, ::defaultQuality);
    d->onQualityChangeRequestImpl(quality);

    video_source::PointF msg;
    msg.x = ::width / ::fieldOfViewH;
    msg.y = ::height / ::fieldOfViewV;
    d->dotsPerDegreePublisher.publish(msg);
    ros::param::set("camera/dotsPerDegreeH", msg.x);
    ros::param::set("camera/dotsPerDegreeV", msg.y);

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
    while (!d->stop)
    {
        cv::Mat frame;
#ifdef PICAM
        d->capturer.grab();
        d->capturer.retrieve(frame);
#else
         d->capturer >> frame;
#endif //PICAM
        if(frame.empty()) continue;

        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        d->imagePublisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
