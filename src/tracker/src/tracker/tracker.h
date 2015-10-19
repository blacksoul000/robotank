#ifndef TRACKER_H
#define TRACKER_H

#include <opencv2/highgui/highgui.hpp>

namespace cv
{
    class Mat;
}

namespace va
{
    class Tracker
    {
    public:
        Tracker();
        ~Tracker();

        void start(const cv::Rect& rect);
        void stop();
        bool isTracking() const;
        void track(cv::Mat& image);
        cv::Rect target() const;

    private:
        class Impl;
        Impl* d;
    };
}

#endif //TRACKER_H
