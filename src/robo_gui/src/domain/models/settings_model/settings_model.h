#ifndef SETTINGS_MODEL_H
#define SETTINGS_MODEL_H

#include <QObject>

namespace domain
{
    class SettingsModel : public QObject
    {
        Q_OBJECT
    public:
        SettingsModel(QObject* parent = nullptr);
        ~SettingsModel();

        void setQuality(int quality);
        int quality() const;

        void setTracker(int tracker);
        int tracker() const;

        void setBrightness(int brightness);
        int brightness() const;

        void setContrast(int contrast);
        int contrast() const;

    signals:
        void qualityChanged(int quality);
        void brightnessChanged(int brightness);
        void contrastChanged(int contrast);
        void trackerChanged(int tracker);

        void calibrateGun();
        void calibrateCamera();
        void calibrateGyro();

    private:
        class Impl;
        Impl* d;
    };
}

#endif //SETTINGS_MODEL_H
