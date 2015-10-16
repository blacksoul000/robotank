#include "presenter_factory.h"
#include "robo_model.h"

#include "frame_presenter.h"
#include "track_presenter.h"

#include <QtQml>

using presentation::PresenterFactory;
using presentation::FramePresenter;

class PresenterFactory::Impl
{
public:
    domain::RoboModel* model;
};

PresenterFactory::PresenterFactory(domain::RoboModel* model, QObject* parent) :
    QObject(parent),
    d(new Impl)
{
    d->model = model;
}

PresenterFactory::~PresenterFactory()
{
    delete d;
}

QObject* PresenterFactory::framePresenter()
{
    return new FramePresenter(d->model, this);
}

QObject* PresenterFactory::trackPresenter()
{
    return new TrackPresenter(d->model, this);
}
