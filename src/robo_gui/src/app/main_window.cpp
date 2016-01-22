#include "main_window.h"

//internal
#include "robo_model.h"
#include "sight_model.h"
#include "track_model.h"
#include "settings_model.h"
#include "presenter_factory.h"

//msgs
#include "tracker/Rect.h"
#include "gamepad_controller/JsEvent.h"

//ros
#include <ros/ros.h>
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
#include <QTimer>
#include <QDebug>

#ifdef ANDROID
#include <QtAndroid>
#include <QAndroidJniObject>
#endif

using robo::MainWindow;

namespace
{
    const QString settingsFileName = "robotank.cfg";
    const QString qualityId = "quality";
    const int defaultQuality = 15;

    const QString trackerId = "tracker";
    const int defaultTracker = 0;

    const std::string transport = "compressed";
    const int imageTimeout = 1000; //ms
}

class MainWindow::Impl
{
public:
    Impl(ros::NodeHandle* nh) : nh(nh)
    {
        imageQualityPub = nh->advertise< std_msgs::UInt8 >("camera/image/quality", 0);
        trackSelectorPub = nh->advertise< std_msgs::UInt8 >("tracker/selector", 1);
        trackPub = nh->advertise< tracker::Rect >("tracker/toggle", 1);
        trackSub = nh->subscribe("tracker/target", 1,
                   &MainWindow::Impl::onNewTarget, this);
        trackStatusSub = nh->subscribe("tracker/status", 1,
                                       &MainWindow::Impl::onTrackerStatusChanged, this);
        gamepadButtonsSub = nh->subscribe("gamepad/buttons", 20,
                                          &MainWindow::Impl::onButtonPressed, this);
    }
    ros::NodeHandle* nh;

    ros::Publisher trackPub;
    ros::Publisher trackSelectorPub;
    ros::Publisher imageQualityPub;
    ros::Subscriber trackSub;
    ros::Subscriber trackStatusSub;
    ros::Subscriber gamepadButtonsSub;
#ifdef ANDROID
    QAndroidJniObject wakeLock;
    bool wakeLocked = false;
//    image_transport's plugin system is not work because of the static ros libs
//    so we need to hardcode using transport
//    and I'm failed to compile hardcoded transport at desktop =(
    compressed_image_transport::CompressedSubscriber imageSub;
#else
    image_transport::Subscriber imageSub;
#endif

    domain::RoboModel* robo = nullptr;
    QQuickView* viewer = nullptr;
    QSettings* settings = nullptr;
    QTimer imageTimer;

    void onNewTarget(const tracker::RectPtr &rect);
    void onTrackerStatusChanged(const std_msgs::UInt8& status);
    QImage mat2QImage(const cv::Mat& image) const;
    void onTrackRequest(const QRectF& rect);
    void onButtonPressed(const gamepad_controller::JsEvent& event);
};

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

void MainWindow::Impl::onTrackRequest(const QRectF& rect)
{
    qDebug() << Q_FUNC_INFO << rect;
    tracker::RectPtr r(new tracker::Rect);
    r->x = rect.x();
    r->y = rect.y();
    r->width = rect.width();
    r->height = rect.height();
    trackPub.publish(r);
}

void MainWindow::Impl::onButtonPressed(const gamepad_controller::JsEvent& event)
{
//    ROS_WARN("type = %d, number = %d, value = %d", event.type, event.number, event.value);
    switch (event.number)
    {
    case 0: //square
        if (event.value == 0)
        {
            QRectF r = robo->track()->isTracking() ? QRectF() : robo->track()->captureRect();
            this->onTrackRequest(r);
        }
        break;
    case 3: //triangle
        if (event.value == 0) robo->track()->nextCaptureSize();
        break;
    default:
        break;
    }
}

void MainWindow::Impl::onTrackerStatusChanged(const std_msgs::UInt8& status)
{
    robo->track()->setTracking(status.data == 1);
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
    d->viewer->requestActivate();

    d->imageTimer.setInterval(::imageTimeout);
    connect(&d->imageTimer, &QTimer::timeout, this, &MainWindow::onImageTimeout);

#ifdef ANDROID
    d->imageSub.subscribe(*d->nh, "camera/image", 1,
                          boost::bind(&MainWindow::onNewFrame, this, _1));
#else
    image_transport::ImageTransport it(*d->nh);
    d->imageSub = it.subscribe("camera/image", 1,
                   boost::bind(&MainWindow::onNewFrame, this, _1), ros::VoidPtr(), transport);
#endif

    this->connectStatusModel();
    this->connectTrackModel();
    this->connectSettingsModel();

    connect(d->viewer->engine(), &QQmlEngine::quit, qApp, &QCoreApplication::quit);
    connect(this, &MainWindow::frameReceived, this, &MainWindow::onFrameReceived);

#ifdef ANDROID
    QAndroidJniObject activity = QtAndroid::androidActivity();

    QAndroidJniObject serviceName = QAndroidJniObject::getStaticObjectField<jstring>(
                "android/content/Context","POWER_SERVICE");
    QAndroidJniObject powerMgr = activity.callObjectMethod("getSystemService",
                                            "(Ljava/lang/String;)Ljava/lang/Object;",
                                            serviceName.object<jobject>());
    QAndroidJniObject tag = QAndroidJniObject::fromString("Robotank");
    d->wakeLock = powerMgr.callObjectMethod("newWakeLock",
                                        "(ILjava/lang/String;)Landroid/os/PowerManager$WakeLock;",
                                        6, //SCREEN_DIM_WAKE_LOCK
                                        tag.object<jstring>());
#endif

}

MainWindow::~MainWindow()
{
    d->imageTimer.stop();
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
    connect(d->robo->settings(), &domain::SettingsModel::qualityChanged,
            this, &MainWindow::onChangeVideoQuality);
    connect(d->robo->settings(), &domain::SettingsModel::trackerChanged,
            this, &MainWindow::onChangeTracker);
}

void MainWindow::onChangeVideoQuality(int quality)
{
    std_msgs::UInt8 msg;
    msg.data = quality;
    d->imageQualityPub.publish(msg);
    ros::param::set("camera/image/quality", quality);

    if (d->settings) d->settings->setValue(::qualityId, quality);
}

void MainWindow::onTrackRequest(const QRectF& rect)
{
    d->onTrackRequest(rect);
}

void MainWindow::onChangeTracker(int tracker)
{
    std_msgs::UInt8 msg;
    msg.data = tracker;
    d->trackSelectorPub.publish(msg);
    ros::param::set("tracker/code", tracker);

    if (d->settings) d->settings->setValue(::trackerId, tracker);
}

void MainWindow::loadSettings()
{
    if (d->settings) return;
    d->settings = new QSettings(::settingsFileName, QSettings::NativeFormat);

    d->robo->settings()->setQuality(d->settings->value(::qualityId, ::defaultQuality).toInt());
    d->robo->settings()->setTracker(d->settings->value(::trackerId, ::defaultTracker).toInt());
}

void MainWindow::onImageTimeout()
{
    d->imageTimer.stop();
    d->robo->sight()->setFrame(QImage());

#ifdef ANDROID
    if (d->wakeLocked)
    {
        d->wakeLocked = false;
        d->wakeLock.callMethod<void>("release", "()V");
    }
#endif
}

void MainWindow::onNewFrame(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat frame =  cv_bridge::toCvShare(msg, "rgb8")->image;
    d->robo->sight()->setFrame(d->mat2QImage(frame));

    emit frameReceived();
}

void MainWindow::onFrameReceived()
{
    d->imageTimer.start();

#ifdef ANDROID
    if (!d->wakeLocked)
    {
        d->wakeLocked = true;
        d->wakeLock.callMethod<void>("acquire", "()V");
    }
#endif
}
