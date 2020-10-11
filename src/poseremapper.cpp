#include "poseremapper.h"
#include <utility>

PoseRemapper::PoseRemapper(QObject *parent) : QObject(parent)
{
    Rcam << 0, 0, -1,
           -1, 0, 0,
            0, 1, 0;
}


void PoseRemapper::on_pose_data(double t, Pose pose) {
    if(!is_inited) {
        initial_pose = pose;
        is_inited = true;
    }

    auto R = initial_pose.first.transpose() * pose.first;
    Eigen::Vector3d T = pose.second - initial_pose.second;

//    auto R = pose.first;
    T = Rcam.transpose() * T;

    auto eul = R2ypr(R);
    this->send_mapped_posedata(t, std::make_pair(eul, T));
}

void PoseRemapper::reset_center() {
    is_inited = false;
}
