#include "tracker_node.h"
#include "tracker_factory.h"
#include "trackers.h"

#include "tracker/Rect.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/video/tracking.hpp"

using va::TrackerNode;

class TrackerNode::Impl
{
public:
    Impl(ros::NodeHandle* nh) : it(*nh)
    {
        toggleSub = nh->subscribe("tracker/toggle", 1,
                   &TrackerNode::Impl::onToggleRequest, this);

        targetPub = nh->advertise< tracker::Rect >("tracker/target", 1);
    }
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub = it.subscribe("camera/image", 1,
                    boost::bind(&TrackerNode::Impl::onNewFrame, this, _1));
    ros::Subscriber toggleSub;
    ros::Publisher targetPub;

    bool tracking = false;
    va::ITracker* tracker = nullptr;
    robotank::TrackerCode trackAlgo = robotank::TrackerCode::Unknown;

    void onNewFrame(const sensor_msgs::ImageConstPtr& msg);
    void onToggleRequest(const tracker::RectPtr &rect);

    void publishTarget(const cv::Rect& rect);
};

TrackerNode::TrackerNode(ros::NodeHandle* nh) :
    d(new Impl(nh))
{
}

TrackerNode::~TrackerNode()
{
    delete d->tracker;
    delete d;
}

void TrackerNode::Impl::onNewFrame(const sensor_msgs::ImageConstPtr &msg)
{
    if (!tracker || !tracker->isTracking()) return;
    cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    tracker->track(frame);

    this->publishTarget(tracker->target());
}

void TrackerNode::Impl::onToggleRequest(const tracker::RectPtr& rect)
{
    ROS_WARN("toggle %f, %f, %f, %f", rect->x, rect->y, rect->width, rect->height);
    bool start = (rect->width != 0 && rect->height != 0);
    if (tracker && tracker->isTracking() == start) return;

    if (start)
    {
        cv::Rect cvRect(rect->x, rect->y, rect->width, rect->height);
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
}

//void TrackerNode::Impl::onSwitchTrackerRequest(const tracker::SwitchTrackerPtr& request)
//{

//}

void TrackerNode::Impl::publishTarget(const cv::Rect& rect)
{
    tracker::RectPtr r(new tracker::Rect);
    r->x = rect.x;
    r->y = rect.y;
    r->width = rect.width;
    r->height = rect.height;
    targetPub.publish(r);
}
