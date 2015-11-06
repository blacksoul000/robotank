#ifndef ROBO_MODEL_H
#define ROBO_MODEL_H

namespace domain
{
    class SightModel;
    class StatusModel;
    class TrackModel;
    class SettingsModel;
    class JoystickModel;

    class RoboModel
    {
    public:
        RoboModel();
        ~RoboModel();
        SightModel* sight() const;
//        StatusModel* status() const;
        TrackModel* track() const;
        SettingsModel* settings() const;
        JoystickModel* joystick() const;

    private:
        SightModel* m_sight = nullptr;
        StatusModel* m_status = nullptr;
        TrackModel* m_track = nullptr;
        SettingsModel* m_settings = nullptr;
        JoystickModel* m_joystick = nullptr;
    };
}

#endif //ROBO_MODEL_H
