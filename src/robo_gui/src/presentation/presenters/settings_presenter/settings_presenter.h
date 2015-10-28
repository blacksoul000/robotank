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
        Q_PROPERTY(qreal quality READ quality WRITE setQuality NOTIFY qualityChanged)

        SettingsPresenter(domain::RoboModel* model, QObject* parent = nullptr);
        ~SettingsPresenter() override;

        qreal quality() const;

    public slots:
        void setQuality(qreal quality) const;

    signals:
        void qualityChanged(qreal quality);

    private:
        class Impl;
        Impl* d;
    };
}

#endif //SETTINGS_PRESENTER_H
