#include "main_window.h"

//internal
#include "robo_model.h"
#include "sight_model.h"
#include "track_model.h"
#include "settings_model.h"
#include "presenter_factory.h"

//msgs
#include "tracker/Rect.h"
#include "tracker/TrackerSelector.h"

//ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/UInt8.h"

// opencv
#include <opencv2/highgui/highgui.hpp>

//Qt
#include <QCoreApplication>
#include <QQmlContext>
#include <QQuickView>
#include <QQmlEngine>
#include <QSettings>
#include <QProcess>
#include <QDebug>

using robo::MainWindow;

namespace
{
    const QString settingsFileName = "robotank.cfg";
    const QString qualityId = "quality";
    const int defaultQuality = 15;

    const QString trackerId = "tracker";
    const int defaultTracker = 0;

    const std::string transport = "compressed";
}

class MainWindow::Impl
{
public:
    Impl(ros::NodeHandle* nh) : nh(nh)//, it(*nh)
    {
        imageQualityPub = nh->advertise< std_msgs::UInt8 >("camera/image/quality", 0);
        trackSelectorPub = nh->advertise< tracker::TrackerSelector >("tracker/selector", 1);
        trackPub = nh->advertise< tracker::Rect >("tracker/toggle", 1);
        trackSub = nh->subscribe("tracker/target", 1,
                   &MainWindow::Impl::onNewTarget, this);
#ifdef ANDROID
        imageSub.subscribe(*nh, "camera/image", 1, boost::bind(&MainWindow::Impl::onNewFrame, this, _1));
#else
        image_transport::ImageTransport it(*nh);
        imageSub = it.subscribe("camera/image", 1,
                   boost::bind(&MainWindow::Impl::onNewFrame, this, _1), ros::VoidPtr(), transport);
#endif
    }
    ros::NodeHandle* nh;

    ros::Publisher trackPub;
    ros::Publisher trackSelectorPub;
    ros::Publisher imageQualityPub;
    ros::Subscriber trackSub;
#ifdef ANDROID
//    image_transport's plugin system is not work because of the static ros libs
//    so we need to hardcode using transport
//    and I'm failed to compile hardcoded transport at desktop =(
    compressed_image_transport::CompressedSubscriber imageSub;
#else
//    image_transport::ImageTransport it;
    image_transport::Subscriber imageSub;
#endif

    domain::RoboModel* robo = nullptr;
    QQuickView* viewer = nullptr;
    QSettings* settings = nullptr;

    void onNewFrame(const sensor_msgs::ImageConstPtr& msg);
    void onNewTarget(const tracker::RectPtr &rect);
    QImage mat2QImage(const cv::Mat& image) const;
};

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

//-----------------------------------------------------------------------
MainWindow::MainWindow(ros::NodeHandle* nh, QObject* parent) :
    QObject(parent),
    d(new Impl(nh))
{
    d->robo = new domain::RoboModel;
    d->viewer = new QQuickView;
    d->viewer->rootContext()->setContextProperty("factory",
                        new presentation::PresenterFactory(d->robo, this));
    d->viewer->setSource(QUrl("qrc:/qml/Main.qml"));
    d->viewer->showFullScreen();

    this->connectStatusModel();
    this->connectTrackModel();
    this->connectSettingsModel();

    connect(d->viewer->engine(), &QQmlEngine::quit, qApp, &QCoreApplication::quit);
}

MainWindow::~MainWindow()
{
    d->settings->sync();
    delete d->settings;
    delete d->viewer;
    delete d->robo;
    delete d;
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

void MainWindow::onChangeVideoQuality(int quality)
{
    std_msgs::UInt8 msg;
    msg.data = quality;
    d->imageQualityPub.publish(msg);
    if (d->settings) d->settings->setValue(::qualityId, quality);
    ros::param::set("camera/image/quality", quality);
}

void MainWindow::onChangeTracker(int tracker)
{
    tracker::TrackerSelectorPtr t(new tracker::TrackerSelector);
    t->code = tracker;
    d->trackSelectorPub.publish(t);

    if (d->settings) d->settings->setValue(::trackerId, tracker);
}

void MainWindow::loadSettings()
{
    if (d->settings) return;
    d->settings = new QSettings(::settingsFileName, QSettings::NativeFormat);

    d->robo->settings()->setQuality(d->settings->value(::qualityId, ::defaultQuality).toInt());
    d->robo->settings()->setTracker(d->settings->value(::trackerId, ::defaultTracker).toInt());
}
