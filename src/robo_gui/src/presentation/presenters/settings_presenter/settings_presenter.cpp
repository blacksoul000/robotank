#include "settings_presenter.h"

#include "robo_model.h"
#include "settings_model.h"
#include "trackers.h"

#include <QSettings>
#include <QRect>
#include <QDebug>

using presentation::SettingsPresenter;

namespace
{
    const QString settingsFileName = "robotank.cfg";
    const QString qualityId = "quality";
    const int defaultQuality = 15;

    const QString trackerId = "tracker";
    const int defaultTracker = int(robotank::TrackerCode::CamShift);
}

class SettingsPresenter::Impl
{
public:
    domain::RoboModel* model = nullptr;
    qreal quality = -1;
    int trackerCode = -1;

    QSettings* settings = nullptr;
};

SettingsPresenter::SettingsPresenter(domain::RoboModel *model, QObject *parent) :
    QObject(parent),
    d(new Impl)
{
    d->model = model;



}

SettingsPresenter::~SettingsPresenter()
{
    d->settings->sync();
    delete d->settings;
    delete d;
}

qreal SettingsPresenter::quality() const
{
    return d->quality;
}

int SettingsPresenter::trackerCode() const
{
    return d->trackerCode;
}

void SettingsPresenter::setQuality(qreal quality)
{
    if (qFuzzyCompare(d->quality, quality)) return;
    d->quality = quality;
    d->model->settings()->setQuality(quality);
    if (d->settings) d->settings->setValue(::qualityId, quality);
}

void SettingsPresenter::setTrackerCode(int code)
{
    if (d->trackerCode == code) return;
    d->trackerCode = code;
    d->model->settings()->setTracker(code);
    if (d->settings) d->settings->setValue(::trackerId, code);
}

void SettingsPresenter::loadSettings()
{
    if (d->settings) return;
    d->settings = new QSettings(::settingsFileName, QSettings::NativeFormat);

    this->setQuality(d->settings->value(::qualityId, ::defaultQuality).toInt());
    this->setTrackerCode(d->trackerCode = d->settings->value(::trackerId, ::defaultTracker).toInt());
    emit qualityChanged(d->quality);
    emit trackerCodeChanged(d->trackerCode);
}
