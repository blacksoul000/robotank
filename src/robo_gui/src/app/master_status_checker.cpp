#include "master_status_checker.h"

#include <QSettings>
#include <QTcpSocket>
#include <QQuickView>
#include <QQuickItem>
#include <QHostAddress>
#include <QDebug>

using app::MasterStatusChecker;

namespace
{
    const QString masterIpId = "masterIp";
    const QString defaultMasterIp = "192.168.1.3";
    const quint16 masterPort = 11311;
}

class MasterStatusChecker::Impl
{
public:
    QQuickView* viewer = nullptr;
    QSettings* settings = nullptr;
    QTcpSocket socket;

    QString selectedIp;
};

MasterStatusChecker::MasterStatusChecker(QSettings* settings, QObject* parent) :
    QObject(parent),
    d(new Impl)
{
    d->settings = settings;

    d->viewer = new QQuickView;
    d->viewer->setSource(QUrl("qrc:/qml/MasterStatus.qml"));
    d->viewer->requestActivate();

    connect(d->viewer->rootObject(), SIGNAL(tryConnect(const QString&)),
            this, SLOT(connectToHost(const QString&)));
    connect(d->viewer->rootObject(), SIGNAL(exit()), qApp, SLOT(quit()));
    connect(&d->socket, &QTcpSocket::connected, this, &MasterStatusChecker::onConnected);

}

MasterStatusChecker::~MasterStatusChecker()
{
    delete d->viewer;
    d->socket.disconnectFromHost();
    delete d;
}

void MasterStatusChecker::start()
{
    d->selectedIp = d->settings->value(::masterIpId, ::defaultMasterIp).toString();
    d->viewer->rootObject()->setProperty("masterIp", d->selectedIp);
    d->viewer->show();
    this->connectToHost(d->selectedIp);
}

QString MasterStatusChecker::ip() const
{
    return d->selectedIp;
}

void MasterStatusChecker::connectToHost(const QString& ip)
{
    QHostAddress host(ip);
    if (host.isNull()) return;
    d->selectedIp = ip;
    d->socket.connectToHost(host, ::masterPort, QIODevice::ReadOnly);
}

void MasterStatusChecker::onConnected()
{
    d->settings->setValue(::masterIpId, d->selectedIp);
    emit connected();
}
