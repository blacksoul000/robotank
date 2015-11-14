#ifndef TRACK_PRESENTER_H
#define TRACK_PRESENTER_H

#include <QObject>
#include <QRectF>

namespace domain
{
    class RoboModel;
}

class QRect;
namespace presentation
{
    class TrackPresenter : public QObject
    {
        Q_OBJECT
    public:
        Q_PROPERTY(QRectF targetRect READ targetRect NOTIFY targetRectChanged);

        TrackPresenter(domain::RoboModel* model, QObject* parent = nullptr);
        ~TrackPresenter() override;

        QRectF targetRect() const;

    public slots:
        void onTrackRequest(const QRectF& rect);

    signals:
        void targetRectChanged(const QRectF& rect);

    private slots:
        void onTargetRectChanged(const QRectF& rect);

    private:
        class Impl;
        Impl* d;
    };
}

#endif //TRACK_PRESENTER_H
