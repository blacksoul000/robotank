#ifndef TRACK_MODEL_H
#define TRACK_MODEL_H

#include <QObject>
#include <QRectF>

namespace domain
{
    class TrackModel : public QObject
    {
        Q_OBJECT
    public:
        TrackModel(QObject* parent = nullptr);
        ~TrackModel();

        void setTargetRect(const QRectF& rect);
        QRectF targetRect() const;

    signals:
        void targetRectChanged(const QRectF& rect);
        void trackRequest(const QRectF& rect);

    private:
        class Impl;
        Impl* d;
    };
} //namespace domain

#endif //TRACK_MODEL_H
