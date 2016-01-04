#ifndef TRACKERS_H
#define TRACKERS_H

namespace va
{
    enum class TrackerCode
    {
        Unknown,
        CamShift,
        MedianFlow,
        Boosting,
        Mil,
        Tld,
        CustomTld
    };
}

#endif //TRACKERS_H
