#include "poseremapper.h"
#include <utility>

PoseRemapper::PoseRemapper(QObject *parent) : QObject(parent)
{
    Rcam << 0, 0, -1,
           -1, 0, 0,
            0, 1, 0;
}


void PoseRemapper::on_pose_data(double t, Pose_ pose_) {
    Pose pose(pose_.second, pose_.first);
    if(!is_inited) {
        initial_pose = pose;
        is_inited = true;
    }

    initial_pose.print();

    pose.print();

    auto Q = initial_pose.att() * pose.att();
    Eigen::Vector3d T = pose.pos() - initial_pose.pos();

    auto eul = quat2eulers(Q);
    this->send_mapped_posedata(t, std::make_pair(eul, T));
}

void PoseRemapper::reset_center() {
    is_inited = false;
}
