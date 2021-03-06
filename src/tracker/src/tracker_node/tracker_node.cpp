#include "tracker_node.h"
#include "tracker_factory.h"
#include "trackers.h"

//msgs
#include "tracker/RectF.h"
#include "tracker/PointF.h"
#include "std_msgs/UInt8.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"

#include <chrono>

using va::TrackerNode;

namespace
{
    const double width = 240;
    const double height = 192;
    const int defaultTracker = 0;
}  // namespace

class TrackerNode::Impl
{
public:
    ros::Subscriber toggleSub;
    ros::Subscriber selectorSub;
    image_transport::Subscriber imageSub;
    ros::Publisher targetPub;
    ros::Publisher deviationPub;
    ros::Publisher trackerStatusPub;

    va::ITracker* tracker = nullptr;
    va::TrackerCode trackAlgo = va::TrackerCode(::defaultTracker);
    int imageWidth = 0;
    int imageHeight = 0;
    int64_t prevTime = 0;

    void onNewFrame(const sensor_msgs::ImageConstPtr& msg);
    void onToggleRequest(const tracker::RectFPtr& rect);
    void onSwitchTrackerRequest(const std_msgs::UInt8& code);

    void publishTarget(const cv::Rect& rect);
    void publishDeviation(const cv::Rect& rect);
    void onTrackerStatusChanged(bool tracking);
};

TrackerNode::TrackerNode(ros::NodeHandle* nh) :
//    d(new Impl(nh))
    d(new Impl)
{
    int code = 0;
    ros::param::param< int >("tracker/code", code, ::defaultTracker);
    d->trackAlgo = va::TrackerCode(code);
    d->targetPub.publish(tracker::RectF());
    d->onTrackerStatusChanged(false);

    image_transport::ImageTransport it(*nh);
    d->imageSub = it.subscribe("camera/image", 1,
                               boost::bind(&TrackerNode::Impl::onNewFrame, d, _1));
    d->toggleSub = nh->subscribe("tracker/toggle", 1, &TrackerNode::Impl::onToggleRequest, d);
    d->selectorSub = nh->subscribe("tracker/selector", 1,
                                   &TrackerNode::Impl::onSwitchTrackerRequest, d);

    d->targetPub = nh->advertise< tracker::RectF >("tracker/target", 1);
    d->deviationPub = nh->advertise< tracker::PointF >("tracker/deviation", 1);
    d->trackerStatusPub = nh->advertise< std_msgs::UInt8 >("tracker/status", 1);
}

TrackerNode::~TrackerNode()
{
    delete d->tracker;
    delete d;
}

void TrackerNode::Impl::onNewFrame(const sensor_msgs::ImageConstPtr &msg)
{
    int64_t s = (std::chrono::duration_cast< std::chrono::microseconds >(
                    std::chrono::system_clock::now().time_since_epoch()).count());

    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    imageWidth = frame.cols;
    imageHeight = frame.rows;
    if (!tracker || !tracker->isTracking()) return;
    cv::Mat scaled;
    resize(frame, scaled, cv::Size(::width, ::height));
    tracker->track(scaled);

    this->publishTarget(tracker->target());
    this->publishDeviation(tracker->target());

    int64_t e = (std::chrono::duration_cast< std::chrono::microseconds >(
                    std::chrono::system_clock::now().time_since_epoch()).count());
    double fps = 1000000.0 / (e - prevTime);
    ROS_WARN("Fps: %f, Updated in: %ld, Real: %ld us", fps, (e - s), (e - prevTime));
    prevTime = e;
}

void TrackerNode::Impl::onToggleRequest(const tracker::RectFPtr& rect)
{
    ROS_WARN("toggle %f, %f, %f, %f", rect->x, rect->y, rect->width, rect->height);
    bool start = (rect->width != 0 && rect->height != 0);
    if (tracker && tracker->isTracking() == start) return;

    if (start)
    {
        const double scaleX = ::width / imageWidth;
        const double scaleY = ::height / imageHeight;
        cv::Rect cvRect(rect->x * scaleX, rect->y * scaleY,
                        rect->width * scaleX, rect->height * scaleY);
        tracker = va::TrackerFactory::makeTracker(trackAlgo);
        tracker->start(cvRect);
    }
    else if(tracker)
    {
        tracker->stop();
        delete tracker;
        tracker = nullptr;

        targetPub.publish(rect);
    }

    this->onTrackerStatusChanged(tracker && tracker->isTracking());
}

void TrackerNode::Impl::onSwitchTrackerRequest(const std_msgs::UInt8& code)
{
    trackAlgo = va::TrackerCode(code.data);
}

void TrackerNode::Impl::publishTarget(const cv::Rect& rect)
{
    const double scaleX = ::width / imageWidth;
    const double scaleY = ::height / imageHeight;

    tracker::RectFPtr r(new tracker::RectF);
    r->x = rect.x / scaleX;
    r->y = rect.y / scaleY;
    r->width = rect.width / scaleX;
    r->height = rect.height / scaleY;
    targetPub.publish(r);
}

void TrackerNode::Impl::publishDeviation(const cv::Rect& rect)
{
    const double scaleX = ::width / imageWidth;
    const double scaleY = ::height / imageHeight;
    const double targetCenterX = (rect.x + rect.width) / 2;
    const double targetCenterY = (rect.y + rect.height) / 2;
    const double imageCenterX = imageWidth / 2;
    const double imageCenterY = imageHeight / 2;

    tracker::PointFPtr r(new tracker::PointF);
    r->x = imageCenterX - (targetCenterX / scaleX);
    r->y = imageCenterY - (targetCenterY / scaleY);
    deviationPub.publish(r);
}

void TrackerNode::Impl::onTrackerStatusChanged(bool tracking)
{
    std_msgs::UInt8 msg;
    msg.data = tracking;
    trackerStatusPub.publish(msg);
}
