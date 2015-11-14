#include "track_presenter.h"

#include "robo_model.h"
#include "track_model.h"

#include <QDebug>

using presentation::TrackPresenter;

class TrackPresenter::Impl
{
public:
    domain::RoboModel* model = nullptr;
    QRectF rect;
};

TrackPresenter::TrackPresenter(domain::RoboModel *model, QObject *parent) :
    QObject(parent),
    d(new Impl)
{
    d->model = model;
    connect(model->track(), &domain::TrackModel::targetRectChanged,
            this, &TrackPresenter::onTargetRectChanged);
}

TrackPresenter::~TrackPresenter()
{
    delete d;
}

void TrackPresenter::onTargetRectChanged(const QRectF& rect)
{
    d->rect = rect;
    emit targetRectChanged(rect);
}

void TrackPresenter::onTrackRequest(const QRectF& rect)
{
    d->model->track()->trackRequest(rect);
}

QRectF TrackPresenter::targetRect() const
{
    return d->rect;
}
