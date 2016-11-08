#ifndef MASTER_STATUS_CHECKER_H
#define MASTER_STATUS_CHECKER_H

#include <QObject>

class QSettings;

namespace app
{
    class MasterStatusChecker : public QObject
    {
        Q_OBJECT

    public:
        explicit MasterStatusChecker(QSettings* settings, QObject* parent = 0);
        ~MasterStatusChecker();

        void start();
        QString ip() const;

    signals:
        void connected();

    private slots:
        void connectToHost(const QString& ip);
        void onConnected();

    private:
        class Impl;
        Impl* d;
    };
}

#endif // MASTER_STATUS_CHECKER_H
