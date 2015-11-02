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
        Q_PROPERTY(int trackerCode READ trackerCode WRITE setTrackerCode NOTIFY trackerCodeChanged)

        SettingsPresenter(domain::RoboModel* model, QObject* parent = nullptr);
        ~SettingsPresenter() override;

        int quality() const;
        int trackerCode() const;

    public slots:
        void setQuality(int quality);
        void setTrackerCode(int code);

    signals:
        void qualityChanged(int quality);
        void trackerCodeChanged(int tracker);

    private slots:
        void onModelQualityChanged(int quality);
        void onModelTrackerChanged(int tracker);

    private:
        class Impl;
        Impl* d;
    };
}

#endif //SETTINGS_PRESENTER_H
