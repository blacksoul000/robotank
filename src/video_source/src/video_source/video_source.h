#ifndef VIDEOSOURCE_H
#define VIDEOSOURCE_H

#include <QObject>

namespace ros
{
    class NodeHandle;
}

namespace video
{
    class VideoSource : public QObject
    {
        Q_OBJECT
    public:
        VideoSource(ros::NodeHandle* nh, int fps, QObject* parent = nullptr);
        ~VideoSource();

        bool start(int cameraNumber);
        void stop();

    private slots:
        void capture();

    private:
        class Impl;
        Impl* d;
    };
} //namespace video
#endif // VIDEOSOURCE_H


