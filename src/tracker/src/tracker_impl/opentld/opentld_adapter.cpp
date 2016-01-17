#include "opentld_adapter.h"

#include "TLD.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>

#include <iostream>

using va::OpenTldAdapter;

class OpenTldAdapter::Impl
{
public:
    cv::Rect target;
    bool tracking = false;
    tld::TLD* tracker;

    bool inited = false;
};

OpenTldAdapter::OpenTldAdapter() :
    va::ITracker(),
    d(new Impl)
{
}

OpenTldAdapter::~OpenTldAdapter()
{
    delete d->tracker;
    delete d;
}

void OpenTldAdapter::start(const cv::Rect& rect)
{
    d->tracking = true;
    d->target = rect;
}

void OpenTldAdapter::stop()
{
    d->tracking = false;
}

bool OpenTldAdapter::isTracking() const
{
    return d->tracking;
}

cv::Rect OpenTldAdapter::target() const
{
    return d->target;
}

void OpenTldAdapter::track(const cv::Mat& image)
{
    if (!d->inited)
    {
        std::cout <<  std::endl << "INIT" << std::endl;
        cv::Mat gray;
        cvtColor(image, gray, CV_BGR2GRAY);
        d->tracker = new tld::TLD();
        d->tracker->detectorCascade->setImgSize(gray.cols, gray.rows, gray.step);
        d->tracker->selectObject(gray, &d->target);
        d->inited = true;
    }

    d->tracker->processImage(image);
    d->target = d->tracker->currBB ? *d->tracker->currBB : cv::Rect();
}
