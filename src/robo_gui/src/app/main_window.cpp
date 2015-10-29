#include "main_window.h"

#include "robo_model.h"
#include "sight_model.h"
#include "track_model.h"
#include "settings_model.h"
#include "presenter_factory.h"

#include "tracker/Rect.h"
#include "trackers.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <QCoreApplication>
#include <QQmlContext>
#include <QQuickView>
#include <QQmlEngine>
#include <QDebug>

using robo::MainWindow;

class MainWindow::Impl
{
public:
    Impl(ros::NodeHandle* nh) : nh(nh), it(*nh)
    {
        nh->setParam("image_transport", "compressed");
        trackPub = nh->advertise< tracker::Rect >("tracker/toggle", 1);
        trackSub = nh->subscribe("tracker/target", 1,
                   &MainWindow::Impl::onNewTarget, this);
    }
    ros::NodeHandle* nh;
    image_transport::ImageTransport it;

    image_transport::Subscriber imageSub = it.subscribe("camera/image", 1,
                    boost::bind(&MainWindow::Impl::onNewFrame, this, _1));

    ros::Publisher trackPub;
    ros::Subscriber trackSub;

    domain::RoboModel* robo = nullptr;
    QQuickView* viewer = nullptr;

    void onNewFrame(const sensor_msgs::ImageConstPtr& msg);
    void onNewTarget(const tracker::RectPtr &rect);
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
    d->viewer->setSource(QUrl("qrc:/qml/Main.qml"));
    d->viewer->show();

    this->connectStatusModel();
    this->connectTrackModel();
    this->connectSettingsModel();

    connect(d->viewer->engine(), &QQmlEngine::quit, qApp, &QCoreApplication::quit);
}

MainWindow::~MainWindow()
{
    delete d->viewer;
    delete d->robo;
    delete d;
}

void MainWindow::Impl::onNewFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv::Mat frame =  cv_bridge::toCvShare(msg, "rgb8")->image;
    robo->sight()->setFrame(this->mat2QImage(frame));
}

void MainWindow::Impl::onNewTarget(const tracker::RectPtr& rect)
{
    QRect r(rect->x, rect->y, rect->width, rect->height);
    robo->track()->setTargetRect(r);
}

QImage MainWindow::Impl::mat2QImage(const cv::Mat& image) const
{
    QImage res(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
    return res.convertToFormat(QImage::Format_RGB32);
}

void MainWindow::connectStatusModel()
{
    qDebug() << Q_FUNC_INFO << "Not Impelemented yet";
}

void MainWindow::connectTrackModel()
{
    connect(d->robo->track(), &domain::TrackModel::trackRequest, this, &MainWindow::onTrackRequest);
}

void MainWindow::connectSettingsModel()
{
    connect(d->robo->settings(), &domain::SettingsModel::qualityChanged, this, &MainWindow::onChangeVideoQuality);
    connect(d->robo->settings(), &domain::SettingsModel::trackerChanged, this, &MainWindow::onChangeTracker);
}

void MainWindow::onTrackRequest(const QRectF& rect)
{
    qDebug() << Q_FUNC_INFO << rect;
    tracker::RectPtr r(new tracker::Rect);
    r->x = rect.x();
    r->y = rect.y();
    r->width = rect.width();
    r->height = rect.height();
    d->trackPub.publish(r);
}

void MainWindow::onChangeVideoQuality(int percent)
{
    qDebug() << Q_FUNC_INFO << percent;
    d->nh->setParam("/camera/image/compressed/jpeg_quality", percent);
}

void MainWindow::onChangeTracker(int tracker)
{
    qDebug() << Q_FUNC_INFO << tracker;
}
