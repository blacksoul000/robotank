#ifndef TRACKER_NODE_H
#define TRACKER_NODE_H

namespace ros
{
    class NodeHandle;
}

namespace va
{
    class TrackerNode
    {
    public:
        TrackerNode(ros::NodeHandle* nh);
        ~TrackerNode();

    private:
        class Impl;
        Impl* d;
    };
}

#endif //TRACKER_NODE_H
