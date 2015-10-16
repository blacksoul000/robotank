#ifndef TRACK_MODEL_H
#define TRACK_MODEL_H

#include <QObject>
#include <QRect>

namespace domain
{
    class TrackModel : public QObject
    {
        Q_OBJECT
    public:
        TrackModel(QObject* parent = nullptr);
        ~TrackModel();

        void setTargetRect(const QRect& rect);
        QRect targetRect() const;

    signals:
        void targetRectChanged(const QRect& rect);
        void trackRequest(const QRect& rect);

    private:
        class Impl;
        Impl* d;
    };
} //namespace domain

#endif //TRACK_MODEL_H
