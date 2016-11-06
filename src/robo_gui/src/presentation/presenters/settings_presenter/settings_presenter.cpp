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
    int brightness = -1;
    int contrast = -1;
    int trackerCode = -1;
};

SettingsPresenter::SettingsPresenter(domain::RoboModel *model, QObject *parent) :
    QObject(parent),
    d(new Impl)
{
    d->model = model;
    d->quality = d->model->settings()->quality();
    d->brightness = d->model->settings()->brightness();
    d->contrast = d->model->settings()->contrast();
    d->trackerCode = d->model->settings()->tracker();

    connect(d->model->settings(), &domain::SettingsModel::qualityChanged,
            this, &SettingsPresenter::onModelQualityChanged);
    connect(d->model->settings(), &domain::SettingsModel::brightnessChanged,
            this, &SettingsPresenter::onModelBrightnessChanged);
    connect(d->model->settings(), &domain::SettingsModel::contrastChanged,
            this, &SettingsPresenter::onModelContrastChanged);
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

int SettingsPresenter::brightness() const
{
    return d->brightness;
}

int SettingsPresenter::contrast() const
{
    return d->contrast;
}

int SettingsPresenter::trackerCode() const
{
    return d->trackerCode;
}

void SettingsPresenter::setQuality(int quality)
{
    d->model->settings()->setQuality(quality);
}

void SettingsPresenter::setBrightness(int brightness)
{
    d->model->settings()->setBrightness(brightness);
}

void SettingsPresenter::setContrast(int contrast)
{
    d->model->settings()->setContrast(contrast);
}

void SettingsPresenter::setTrackerCode(int tracker)
{
    d->model->settings()->setTracker(tracker);
}

void SettingsPresenter::onModelQualityChanged(int quality)
{
    if (d->quality == quality) return;
    d->quality = quality;
    emit qualityChanged(quality);
}

void SettingsPresenter::onModelBrightnessChanged(int brightness)
{
    if (d->brightness == brightness) return;
    d->brightness = brightness;
    emit qualityChanged(brightness);
}

void SettingsPresenter::onModelContrastChanged(int contrast)
{
    if (d->contrast == contrast) return;
    d->contrast = contrast;
    emit contrastChanged(contrast);
}

void SettingsPresenter::onModelTrackerChanged(int tracker)
{
    if (d->trackerCode == tracker) return;
    d->trackerCode = tracker;
    emit trackerCodeChanged(tracker);
}

void SettingsPresenter::calibrateGun()
{
    d->model->settings()->calibrateGun();
}

void SettingsPresenter::calibrateCamera()
{
    d->model->settings()->calibrateCamera();
}

void SettingsPresenter::calibrateGyro()
{
    d->model->settings()->calibrateGyro();
}
