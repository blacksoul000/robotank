#include "main_window.h"

#include "robo_model.h"
#include "sight_model.h"
#include "presenter_factory.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QQmlContext>
#include <QQuickView>
#include <QDebug>

using robo::MainWindow;

class MainWindow::Impl
{
public:
    Impl(ros::NodeHandle* nh) : it(*nh) {}
    image_transport::ImageTransport it;
    image_transport::Subscriber imageSubscriber = it.subscribe("camera/image", 1,
                    boost::bind(&MainWindow::Impl::onNewFrame, this, _1));

    domain::RoboModel* robo = nullptr;
    QQuickView* viewer = nullptr;

    void onNewFrame(const sensor_msgs::ImageConstPtr& msg);
    QImage mat2QImage(const cv::Mat& image) const;
};

MainWindow::MainWindow(ros::NodeHandle* nh, QObject* parent) :
    QObject(parent),
    d(new Impl(nh))
{
    d->robo = new domain::RoboModel;
    d->viewer = new QQuickView;
    d->viewer->rootContext()->setContextProperty("factory",
                        new presentation::PresenterFactory(d->robo, this));
    d->viewer->setSource(QUrl("qrc:/qml/main.qml"));
    d->viewer->show();
}

MainWindow::~MainWindow()
{
    delete d->viewer;
    delete d->robo;
    delete d;
}

void MainWindow::Impl::onNewFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat frame =  cv_bridge::toCvShare(msg, "bgr8")->image;
    robo->sight()->setFrame(this->mat2QImage(frame));
}

QImage MainWindow::Impl::mat2QImage(const cv::Mat& image) const
{
    QImage res = QImage(image.cols, image.rows, QImage::Format_RGB32);
    const unsigned char* data = image.data;
    for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
    {
        for(int x = 0; x < image.cols; ++x)
        {
            QRgb* p = ((QRgb*)res.scanLine (y)) + x;
            *p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1],
                    data[x * image.channels()]);
        }
    }
    return res;
}
