#ifndef SETTINGS_PRESENTER_H
#define SETTINGS_PRESENTER_H

#include <QObject>

namespace domain
{
    class RoboModel;
}

namespace presentation
{
    class SettingsPresenter : public QObject
    {
        Q_OBJECT
    public:
        Q_PROPERTY(int quality READ quality WRITE setQuality NOTIFY qualityChanged)
        Q_PROPERTY(int brightness READ brightness WRITE setBrightness NOTIFY brightnessChanged)
        Q_PROPERTY(int contrast READ contrast WRITE setContrast NOTIFY contrastChanged)
        Q_PROPERTY(int trackerCode READ trackerCode WRITE setTrackerCode NOTIFY trackerCodeChanged)

        SettingsPresenter(domain::RoboModel* model, QObject* parent = nullptr);
        ~SettingsPresenter() override;

        int quality() const;
        int brightness() const;
        int contrast() const;
        int trackerCode() const;

    public slots:
        void setQuality(int quality);
        void setBrightness(int brightness);
        void setContrast(int contrast);
        void setTrackerCode(int code);

        void calibrateGun();
        void calibrateCamera();
        void calibrateGyro();

    signals:
        void qualityChanged(int quality);
        void brightnessChanged(int brightness);
        void contrastChanged(int contrast);
        void trackerCodeChanged(int tracker);

    private slots:
        void onModelQualityChanged(int quality);
        void onModelBrightnessChanged(int brightness);
        void onModelContrastChanged(int contrast);
        void onModelTrackerChanged(int tracker);

    private:
        class Impl;
        Impl* d;
    };
}

#endif //SETTINGS_PRESENTER_H
