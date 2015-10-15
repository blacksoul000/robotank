#include "presenter_factory.h"

#include "frame_presenter.h"
#include "robo_model.h"

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
