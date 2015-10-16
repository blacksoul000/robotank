#ifndef ROBO_MODEL_H
#define ROBO_MODEL_H

namespace domain
{
    class SightModel;
    class StatusModel;
    class TrackModel;

    class RoboModel
    {
    public:
        RoboModel();
        ~RoboModel();
        SightModel* sight() const;
//        StatusModel* status() const;
        TrackModel* track() const;

    private:
        SightModel* m_sight = nullptr;
        StatusModel* m_status = nullptr;
        TrackModel* m_track = nullptr;
    };
}

#endif //ROBO_MODEL_H
