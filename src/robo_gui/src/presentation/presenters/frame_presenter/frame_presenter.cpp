#include "frame_presenter.h"

#include "robo_model.h"
#include "sight_model.h"

#include <QVideoSurfaceFormat>
#include <QAbstractVideoSurface>
#include <QDebug>

using presentation::FramePresenter;

class FramePresenter::Impl
{
public:
    domain::RoboModel* model = nullptr;
    QAbstractVideoSurface* surface = nullptr;
    QVideoSurfaceFormat format;
};

FramePresenter::FramePresenter(domain::RoboModel *model, QObject *parent) :
    QObject(parent),
    d(new Impl)
{
    d->model = model;
    connect(model->sight(), &domain::SightModel::frameChanged,
            this, &FramePresenter::onFrameChanged);
}

FramePresenter::~FramePresenter()
{
    delete d;
}

QAbstractVideoSurface *FramePresenter::videoSurface() const
{
    return d->surface;
}

void FramePresenter::setVideoSurface( QAbstractVideoSurface* s )
{
    this->closeSurface();
    d->surface = s;
}

void FramePresenter::closeSurface()
{
    if (d->surface && d->surface->isActive()) d->surface->stop();
}

void FramePresenter::onFrameChanged(const QImage& frame)
{
    if (!d->surface) return;
    if (frame.isNull()) return;

    QVideoFrame::PixelFormat pixelFormat =
            QVideoFrame::pixelFormatFromImageFormat(frame.format());

    if (d->format.frameHeight() != frame.height()
            || d->format.frameWidth() != frame.width()
            || pixelFormat != d->format.pixelFormat())
    {
        this->closeSurface();
        d->format = QVideoSurfaceFormat(frame.size(), pixelFormat);
        d->surface->start(d->format);
    }

    d->surface->present(QVideoFrame(frame));
}
