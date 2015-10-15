#ifndef FRAME_PRESENTER_H
#define FRAME_PRESENTER_H

#include <QImage>
#include <QObject>

namespace domain
{
    class RoboModel;
}

class QAbstractVideoSurface;
namespace presentation
{
    class FramePresenter : public QObject
    {
        Q_OBJECT
    public:
        Q_PROPERTY(QAbstractVideoSurface* videoSurface READ videoSurface
                   WRITE setVideoSurface)

        FramePresenter(domain::RoboModel* model, QObject* parent = nullptr);
        ~FramePresenter() override;

        QAbstractVideoSurface* videoSurface() const;
        void setVideoSurface(QAbstractVideoSurface* s);

    private slots:
        void onFrameChanged(const QImage& frame);

    private:
        void closeSurface();

    private:
        class Impl;
        Impl* d;
    };
}

#endif //FRAME_PRESENTER_H
