#include "settings_presenter.h"

#include "robo_model.h"
#include "settings_model.h"

#include <QRect>
#include <QDebug>

using presentation::SettingsPresenter;

class SettingsPresenter::Impl
{
public:
    domain::RoboModel* model = nullptr;
    int quality = -1;
    int trackerCode = -1;
};

SettingsPresenter::SettingsPresenter(domain::RoboModel *model, QObject *parent) :
    QObject(parent),
    d(new Impl)
{
    d->model = model;
    d->quality = d->model->settings()->quality();
    d->trackerCode = d->model->settings()->tracker();

    connect(d->model->settings(), &domain::SettingsModel::qualityChanged,
            this, &SettingsPresenter::onModelQualityChanged);
    connect(d->model->settings(), &domain::SettingsModel::trackerChanged,
            this, &SettingsPresenter::onModelTrackerChanged);
}

SettingsPresenter::~SettingsPresenter()
{
    delete d;
}

int SettingsPresenter::quality() const
{
    return d->quality;
}

int SettingsPresenter::trackerCode() const
{
    return d->trackerCode;
}

void SettingsPresenter::setQuality(int quality)
{
    if (d->quality == quality) return;
    d->quality = quality;
    d->model->settings()->setQuality(quality);
}

void SettingsPresenter::setTrackerCode(int tracker)
{
    if (d->trackerCode == tracker) return;
    d->trackerCode = tracker;
    d->model->settings()->setTracker(tracker);
}

void SettingsPresenter::onModelQualityChanged(int quality)
{
    if (d->quality == quality) return;
    d->quality = quality;
    emit qualityChanged(quality);
}

void SettingsPresenter::onModelTrackerChanged(int tracker)
{
    if (d->trackerCode == tracker) return;
    d->trackerCode = tracker;
    emit trackerCodeChanged(tracker);
}
