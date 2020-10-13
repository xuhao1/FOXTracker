#include "PoseDataSender.h"

void PoseDataSender::on_pose6d_data(double t, Pose6DoF pose) {
    if(settings->send_posedata_udp) {
        this->send_data_udp(t, pose);
    }

    if(settings->use_ft || settings->use_npclient) {
        ft->on_pose6d_data(t, pose);
    }
}



void PoseDataSender::send_data_udp(double t, Pose6DoF pose) {
    auto eul = pose.first;
    //Send packet
    //Debug only
    double data[6] = {0};
    data[0] = - pose.second.x()*100;
    data[1] = pose.second.y()*100;
    data[2] = - pose.second.z()*100;

    data[3] = eul.x();
    data[4] = eul.y();
    data[5] = -eul.z();

    udpsock->writeDatagram((char*)data, sizeof(double)*6,
                           QHostAddress(settings->udp_host.c_str()), settings->port);
}
