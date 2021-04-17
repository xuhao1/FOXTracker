#include "poseremapper.h"
#include <utility>
#include <QDebug>

PoseRemapper::PoseRemapper(QObject *parent) : QObject(parent)
{
    Rcam << 0, 0, -1,
           -1, 0, 0,
            0, 1, 0;
}


void PoseRemapper::on_pose_data(double t, Pose_ pose_) {
    Pose pose(pose_.second, pose_.first);
    if(!is_inited) {
        qDebug() << "Reset initial pose";
        std::cout << initial_pose.att().toRotationMatrix() << std::endl;
        initial_pose = pose;
        is_inited = true;
    }

    auto Q = initial_pose.att().inverse() * pose.att();
    Eigen::Vector3d T = pose.pos() - initial_pose.pos();
    T = initial_pose.att().inverse() * T;
    auto eul = quat2eulers(Q);
    this->send_mapped_posedata(t, std::make_pair(eul, T));
}

void PoseRemapper::reset_center() {
    is_inited = false;
}
