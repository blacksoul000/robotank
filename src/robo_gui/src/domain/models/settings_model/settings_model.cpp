#include "settings_model.h"

#include <QDebug>

using domain::SettingsModel;

class SettingsModel::Impl
{
public:
    int quality = -1;
    int tracker = -1;
};

SettingsModel::SettingsModel(QObject *parent) :
    QObject(parent),
    d(new Impl)
{
}

SettingsModel::~SettingsModel()
{
    delete d;
}

void SettingsModel::setQuality(int quality)
{
    if (d->quality == quality) return;
    d->quality = quality;
    emit qualityChanged(quality);
}

int SettingsModel::quality() const
{
    return d->quality;
}

void SettingsModel::setTracker(int tracker)
{
    if (d->tracker == tracker) return;
    d->tracker = tracker;
    emit trackerChanged(tracker);
}

int SettingsModel::tracker() const
{
    return d->tracker;
}


