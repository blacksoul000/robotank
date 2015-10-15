#ifndef ROBO_MODEL_H
#define ROBO_MODEL_H

namespace domain
{
    class SightModel;
    class StatusModel;
    class TrackingModel;

    class RoboModel
    {
    public:
        RoboModel();
        ~RoboModel();
        SightModel* sight() const;
//        StatusModel* status() const;
//        TrackingModel* tracking() const;

    private:
        SightModel* m_sight = nullptr;
        StatusModel* m_status = nullptr;
        TrackingModel* m_tracking = nullptr;
    };
}

#endif //ROBO_MODEL_H
