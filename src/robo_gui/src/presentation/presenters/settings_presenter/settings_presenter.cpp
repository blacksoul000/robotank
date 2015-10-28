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
    qreal quality;
};

SettingsPresenter::SettingsPresenter(domain::RoboModel *model, QObject *parent) :
    QObject(parent),
    d(new Impl)
{
    d->model = model;
//    connect(model->track(), &domain::TrackModel::targetRectChanged,
//            this, &SettingsPresenter::onTargetRectChanged);
}

SettingsPresenter::~SettingsPresenter()
{
    delete d;
}

qreal SettingsPresenter::quality() const
{
    return d->quality;
}

void SettingsPresenter::setQuality(qreal quality) const
{
    d->quality = quality;
    d->model->settings()->qualityChanged(quality);
}
