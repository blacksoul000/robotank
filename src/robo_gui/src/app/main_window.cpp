#include "main_window.h"

//internal
#include "robo_model.h"
#include "sight_model.h"
#include "track_model.h"
#include "settings_model.h"
#include "status_model.h"
#include "presenter_factory.h"

//msgs
#include "video_source/ImageSettings.h"
#include "tracker/RectF.h"
#include "gamepad_controller/JsEvent.h"
#include "robo_core/PointF.h"
#include "robo_core/Point3D.h"
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>

//ros
#include <ros/ros.h>
#include <compressed_image_transport/compressed_subscriber.h>
#include <cv_bridge/cv_bridge.h>

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
    const QString qualityId = "quality";
    const QString brightnessId = "brightness";
    const QString contrastId = "contrast";

    const int defaultQuality = 15;
    const int defaultBrightness = 75;
    const int defaultContrast = 95;

    const QString trackerId = "tracker";
    const int defaultTracker = 0;

    const std::string transport = "compressed";
    const int imageTimeout = 1000; //ms
}

class MainWindow::Impl
{
public:
    QList< ros::Subscriber > subscribers;

    ros::Publisher trackPub;
    ros::Publisher trackSelectorPub;
    ros::Publisher imageSettingsPub;
    ros::Publisher calibrateGunPub;
    ros::Publisher calibrateCameraPub;
    ros::Publisher calibrateGyroPub;

#ifdef ANDROID
    QAndroidJniObject wakeLock;
    bool wakeLocked = false;
//    image_transport's plugin system is not work because of the static ros libs
//    so we need to hardcode used transport
//    and I'm failed to compile hardcoded transport at desktop =(
    compressed_image_transport::CompressedSubscriber imageSub;
#else
    image_transport::Subscriber imageSub;
#endif

    domain::RoboModel* model = nullptr;
    QQuickView* viewer = nullptr;
    QSettings* settings = nullptr;
    QTimer imageTimer;

    QImage mat2QImage(const cv::Mat& image) const;

    void onNewTarget(const tracker::RectFPtr &rect);
    void onTrackerStatusChanged(const std_msgs::UInt8& status);
    void onTrackRequest(const QRectF& rect);
    void onButtonPressed(const gamepad_controller::JsEvent& event);
    void onGunPositionChanged(const robo_core::PointF& position);
    void onCameraPositionChanged(const robo_core::PointF& position);
    void onYawPitchRollChanged(const robo_core::Point3D& ypr);
};

MainWindow::MainWindow(ros::NodeHandle* nh, QSettings *settings, QObject* parent) :
    QObject(parent),
    d(new Impl)
{
    d->settings = settings;
    d->model = new domain::RoboModel;
    d->viewer = new QQuickView;
    d->viewer->rootContext()->setContextProperty("factory",
                        new presentation::PresenterFactory(d->model, this));
    d->viewer->setSource(QUrl("qrc:/qml/Main.qml"));
    d->viewer->showFullScreen();
    d->viewer->requestActivate();

    d->imageTimer.setInterval(::imageTimeout);
    connect(&d->imageTimer, &QTimer::timeout, this, &MainWindow::onImageTimeout);

    // publishers
    d->imageSettingsPub = nh->advertise< video_source::ImageSettings >("camera/image/settings", 1);
    d->trackSelectorPub = nh->advertise< std_msgs::UInt8 >("tracker/selector", 1);
    d->trackPub = nh->advertise< tracker::RectF >("tracker/toggle", 1);
    d->calibrateGunPub = nh->advertise< std_msgs::Empty >("gun/calibrate", 1);
    d->calibrateCameraPub = nh->advertise< std_msgs::Empty >("camera/calibrate", 1);
    d->calibrateGyroPub = nh->advertise< std_msgs::Empty >("ypr/calibrate", 1);

    //subscribers
    d->subscribers.append(nh->subscribe("gun/position", 1,
                                        &MainWindow::Impl::onGunPositionChanged, d));
    d->subscribers.append(nh->subscribe("camera/position", 1,
                                      &MainWindow::Impl::onCameraPositionChanged, d));
    d->subscribers.append(nh->subscribe("robo/ypr", 1,
                                        &MainWindow::Impl::onYawPitchRollChanged, d));
    d->subscribers.append(nh->subscribe("tracker/target", 1, &MainWindow::Impl::onNewTarget, d));
    d->subscribers.append(nh->subscribe("tracker/status", 1,
                                        &MainWindow::Impl::onTrackerStatusChanged, d));
    d->subscribers.append(nh->subscribe("gamepad/buttons", 20,
                                        &MainWindow::Impl::onButtonPressed, d));

#ifdef ANDROID
    d->imageSub.subscribe(*nh, "camera/image", 1,
                          boost::bind(&MainWindow::onNewFrame, this, _1));
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
#else
    image_transport::ImageTransport it(*nh);
    d->imageSub = it.subscribe("camera/image", 1,
                   boost::bind(&MainWindow::onNewFrame, this, _1), ros::VoidPtr(), ::transport);
#endif

    this->connectStatusModel();
    this->connectTrackModel();
    this->connectSettingsModel();

    connect(d->viewer->engine(), &QQmlEngine::quit, qApp, &QCoreApplication::quit);
    connect(this, &MainWindow::frameReceived, this, &MainWindow::onFrameReceived);
}

MainWindow::~MainWindow()
{
    d->imageTimer.stop();
    delete d->viewer;
    delete d->model;
    delete d;
}

void MainWindow::connectStatusModel()
{
}

void MainWindow::connectTrackModel()
{
    connect(d->model->track(), &domain::TrackModel::trackRequest, this, &MainWindow::onTrackRequest);
}

void MainWindow::connectSettingsModel()
{
    connect(d->model->settings(), &domain::SettingsModel::qualityChanged,
            this, &MainWindow::onImageSettingsChanged);
    connect(d->model->settings(), &domain::SettingsModel::brightnessChanged,
            this, &MainWindow::onImageSettingsChanged);
    connect(d->model->settings(), &domain::SettingsModel::contrastChanged,
            this, &MainWindow::onImageSettingsChanged);
    connect(d->model->settings(), &domain::SettingsModel::trackerChanged,
            this, &MainWindow::onChangeTracker);
    connect(d->model->settings(), &domain::SettingsModel::calibrateGun,
            this, &MainWindow::onCalibrateGun);
    connect(d->model->settings(), &domain::SettingsModel::calibrateCamera,
            this, &MainWindow::onCalibrateCamera);
    connect(d->model->settings(), &domain::SettingsModel::calibrateGyro,
            this, &MainWindow::onCalibrateGyro);
}

void MainWindow::onImageSettingsChanged()
{
    video_source::ImageSettings msg;

    msg.quality = d->model->settings()->quality();
    msg.brightness = d->model->settings()->brightness();
    msg.contrast = d->model->settings()->contrast();
    d->imageSettingsPub.publish(msg);
    ros::param::set("camera/image/quality", msg.quality);
    ros::param::set("camera/image/brightness", msg.brightness);
    ros::param::set("camera/image/contrast", msg.contrast);

    if (d->settings)
    {
        d->settings->setValue(::qualityId, msg.quality);
        d->settings->setValue(::brightnessId, msg.brightness);
        d->settings->setValue(::contrastId, msg.contrast);
    }
}

void MainWindow::onCalibrateGun()
{
    std_msgs::Empty msg;
    d->calibrateGunPub.publish(msg);
}

void MainWindow::onCalibrateCamera()
{
    std_msgs::Empty msg;
    d->calibrateCameraPub.publish(msg);
}

void MainWindow::onCalibrateGyro()
{
    std_msgs::Empty msg;
    d->calibrateGyroPub.publish(msg);
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
    const int quality = d->settings->value(::qualityId, ::defaultQuality).toInt();
    const int brightness = d->settings->value(::brightnessId, ::defaultBrightness).toInt();
    const int contrast = d->settings->value(::contrastId, ::defaultContrast).toInt();
    const int traker = d->settings->value(::trackerId, ::defaultTracker).toInt();
    d->model->settings()->setQuality(quality);
    d->model->settings()->setBrightness(brightness);
    d->model->settings()->setContrast(contrast);
    d->model->settings()->setTracker(traker);
}

void MainWindow::onImageTimeout()
{
    d->imageTimer.stop();
    d->model->sight()->setFrame(QImage());

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
    d->model->sight()->setFrame(d->mat2QImage(frame));

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


//-------------------------------------------------------------------------------------------------
void MainWindow::Impl::onNewTarget(const tracker::RectFPtr& rect)
{
    QRect r(rect->x, rect->y, rect->width, rect->height);
    model->track()->setTargetRect(r);
}

QImage MainWindow::Impl::mat2QImage(const cv::Mat& image) const
{
    QImage res(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
    return res.convertToFormat(QImage::Format_RGB32);
}

void MainWindow::Impl::onTrackRequest(const QRectF& rect)
{
    qDebug() << Q_FUNC_INFO << rect;
    tracker::RectFPtr r(new tracker::RectF);
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
            QRectF r = model->track()->isTracking() ? QRectF() : model->track()->captureRect();
            this->onTrackRequest(r);
        }
        break;
    case 3: //triangle
        if (event.value == 0) model->track()->nextCaptureSize();
        break;
    default:
        break;
    }
}

void MainWindow::Impl::onTrackerStatusChanged(const std_msgs::UInt8& status)
{
    model->track()->setTargetRect(QRect());
    model->track()->setTracking(status.data == 1);
}

void MainWindow::Impl::onGunPositionChanged(const robo_core::PointF& position)
{
    model->status()->setGunPositionH(position.x);
    model->status()->setGunPositionV(position.y);
}

void MainWindow::Impl::onCameraPositionChanged(const robo_core::PointF& position)
{
    model->status()->setCameraPositionV(position.y);
}

void MainWindow::Impl::onYawPitchRollChanged(const robo_core::Point3D& ypr)
{
    model->status()->setYaw(ypr.x);
    model->status()->setPitch(ypr.y);
    model->status()->setRoll(ypr.z);
}
