#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QObject>

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
        MainWindow(ros::NodeHandle* nh, QObject* parent = nullptr);
        ~MainWindow();

    private:
        void connectStatusModel();
        void connectTrackModel();
        void connectSettingsModel();

    private slots:
        void onTrackRequest(const QRectF& rect);
        void onChangeVideoQuality(int percent);

    private:
        class Impl;
        Impl* d;
    };
}

#endif //MAIN_WINDOW_H