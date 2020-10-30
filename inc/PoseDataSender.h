#ifndef POSEDATASENDER_H
#define POSEDATASENDER_H
#include "freetrack/ftnoir_protocol_ft.h"
#include <QtNetwork>
#include <QDebug>
class PoseDataSender: public QObject {
    QUdpSocket * udpsock = nullptr;
    freetrack * ft = nullptr;

    void send_data_udp(double t, Pose6DoF pose);
public:
    PoseDataSender() {
        udpsock = new QUdpSocket(nullptr);
        ft = new freetrack;
        bool success = ft->initialize();
        if (!success) {
            qDebug("Initialize failed");
        } else {
            qDebug("Initialize freetrack OK");
        }
    }

public slots:
    void on_pose6d_data(double t, Pose6DoF pose);
};

#endif // POSEDATASENDER_H
