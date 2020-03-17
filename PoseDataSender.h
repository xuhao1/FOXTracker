#ifndef POSEDATASENDER_H
#define POSEDATASENDER_H
#include "HeadPoseDetector.h"

class PoseDataSender: public QObject {
    QUdpSocket * udpsock = nullptr;

    void send_data_udp(double t, Pose6DoF pose);
public:
    PoseDataSender() {
        udpsock = new QUdpSocket(nullptr);
    }

public slots:
    void on_pose6d_data(double t, Pose6DoF pose);
};

#endif // POSEDATASENDER_H
