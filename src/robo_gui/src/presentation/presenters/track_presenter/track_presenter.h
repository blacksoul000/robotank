#ifndef TRACK_PRESENTER_H
#define TRACK_PRESENTER_H

#include <QObject>

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
        Q_PROPERTY(QRect targetRect READ targetRect NOTIFY targetRectChanged);

        TrackPresenter(domain::RoboModel* model, QObject* parent = nullptr);
        ~TrackPresenter() override;

        QRect targetRect() const;

    public slots:
        void onTrackRequest(const QRect& rect);

    signals:
        void targetRectChanged(const QRect& rect);

    private slots:
        void onTargetRectChanged(const QRect& rect);

    private:
        class Impl;
        Impl* d;
    };
}

#endif //TRACK_PRESENTER_H
