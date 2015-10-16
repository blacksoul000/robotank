#include "track_model.h"

using domain::TrackModel;

class TrackModel::Impl
{
public:
    QRect targetRect;
};

TrackModel::TrackModel(QObject* parent) :
    QObject(parent),
    d(new Impl)
{
}

TrackModel::~TrackModel()
{
    delete d;
}

void TrackModel::setTargetRect(const QRect& rect)
{
    if (rect == d->targetRect) return;
    d->targetRect = rect;

    emit targetRectChanged(rect);
}

QRect TrackModel::targetRect() const
{
    return d->targetRect;
}
