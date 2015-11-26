#ifndef VIDEOSOURCE_H
#define VIDEOSOURCE_H

namespace ros
{
    class NodeHandle;
}

namespace video
{
    class VideoSource
    {
    public:
        VideoSource(ros::NodeHandle* nh, int fps);
        ~VideoSource();

        void start(int cameraNumber);
        void stop();
        
    private:
        void capture();

    private:
        class Impl;
        Impl* d;
    };
} //namespace video
#endif // VIDEOSOURCE_H


