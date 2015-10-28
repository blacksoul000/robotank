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

    signals:
        void qualityChanged(int quality);

    private:
        class Impl;
        Impl* d;
    };
}

#endif //SETTINGS_MODEL_H
