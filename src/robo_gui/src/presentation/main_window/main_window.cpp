#include "main_window.h"

#include "robo_model.h"
#include "sight_model.h"
#include "track_model.h"
#include "presenter_factory.h"

#include "tracker/Rect.h"

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
    Impl(ros::NodeHandle* nh) : it(*nh) 
    {
        trackPub = nh->advertise< tracker::Rect >("tracker/toggle", 1);
        trackSub = nh->subscribe("tracker/target", 1,
                   &MainWindow::Impl::onNewTarget, this);
    }
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

void MainWindow::Impl::onNewTarget(const tracker::RectPtr& rect)
{
    QRect r(rect->x, rect->y, rect->width, rect->height);
    robo->track()->setTargetRect(r);
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
//    static QVector<QRgb>  sColorTable;

//    // only create our color table once
//    if ( sColorTable.isEmpty() )
//    {
//       for ( int i = 0; i < 256; ++i )
//          sColorTable.push_back( qRgb( i, i, i ) );
//    }

//    QImage res(image.data, image.cols, image.rows, image.step, QImage::Format_Indexed8);
//    res.setColorTable(sColorTable);
//    return res;
}

void MainWindow::connectStatusModel()
{
    qDebug() << Q_FUNC_INFO << "Not Impelemented yet";
}

void MainWindow::connectTrackModel()
{
    connect(d->robo->track(), &domain::TrackModel::trackRequest, this, &MainWindow::onTrackRequest);
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
