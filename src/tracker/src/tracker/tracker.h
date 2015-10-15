#ifndef TRACKER_H
#define TRACKER_H

namespace ros
{
    class NodeHandle;
}

namespace va
{
    class Tracker
    {
    public:
        Tracker(ros::NodeHandle* nh);
        ~Tracker();

    private:
        class Impl;
        Impl* d;
    };
}

#endif //TRACKER_H
