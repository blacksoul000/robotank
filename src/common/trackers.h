#ifndef TRACKERS_H
#define TRACKERS_H

namespace robotank
{
    enum class TrackerCode
    {
        Unknown,
        CamShift,
        MedianFlow,
        Boosting,
        Mil,
        Tld
    };
}

#endif //TRACKERS_H
