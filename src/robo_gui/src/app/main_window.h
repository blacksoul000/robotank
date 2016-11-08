#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <image_transport/image_transport.h>

#include <QObject>

class QSettings;

namespace ros
{
    class NodeHandle;
}

namespace robo
{
    class MainWindow : public QObject
    {
        Q_OBJECT
    public:
        MainWindow(ros::NodeHandle* nh, QSettings* settings, QObject* parent = nullptr);
        ~MainWindow();

        void loadSettings();

    signals:
        void frameReceived();

    private:
        void connectStatusModel();
        void connectTrackModel();
        void connectSettingsModel();

    private slots:
        void onTrackRequest(const QRectF& rect);
        void onImageSettingsChanged();
        void onChangeTracker(int tracker);

        void onCalibrateGun();
        void onCalibrateCamera();
        void onCalibrateGyro();

        void onImageTimeout();

        void onNewFrame(const sensor_msgs::ImageConstPtr& msg);
        void onFrameReceived();

    private:
        class Impl;
        Impl* d;
    };
}

#endif //MAIN_WINDOW_H
