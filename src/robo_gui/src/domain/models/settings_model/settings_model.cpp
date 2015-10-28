#include "settings_model.h"

using domain::SettingsModel;

class SettingsModel::Impl
{
public:
    int quality;
};

SettingsModel::SettingsModel(QObject *parent) :
    QObject(parent),
    d(new Impl)
{
    //TODO - load settings
}

SettingsModel::~SettingsModel()
{
    delete d;
}

void SettingsModel::setQuality(int quality)
{
    d->quality = quality;
    emit qualityChanged(quality);
}

int SettingsModel::quality() const
{
    return d->quality;
}

