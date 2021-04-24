#include "PoseDataSender.h"
#include "FlightAgxSettings.h"
void PoseDataSender::on_pose6d_data(double t, Pose6DoF pose) {
    if(settings->send_posedata_udp) {
        this->send_data_udp(t, pose);
    }

    if(settings->use_ft || settings->use_npclient) {

        //Has bug of conversion here!!! Roll is inverted
        auto _pose = pose;
        pose.first(2) = - pose.first(2);

        pose.second.x() = _pose.second.y();
        pose.second.y() = -_pose.second.z();
        pose.second.z() = _pose.second.x();

        ft->on_pose6d_data(t, pose);
    }
}



void PoseDataSender::send_data_udp(double t, Pose6DoF pose) {
    auto eul = pose.first;
    //Send packet
    //Debug only
    double data[6] = {0};
//    data[0] = pose.second.x()*100;
//    data[1] = - pose.second.y()*100;
//    data[2] = pose.second.z()*100;
    data[0] = -pose.second.y()*100;
    data[1] = -pose.second.z()*100;
    data[2] = -pose.second.x()*100;

    data[3] = eul(0);
    data[4] = eul(1);
    data[5] = -eul(2);

    udpsock->writeDatagram((char*)data, sizeof(double)*6,
                           QHostAddress(settings->udp_host.c_str()), settings->port);
}
