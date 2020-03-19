#include "poseremapper.h"
#include <utility>

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R, int degress = true)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    if (degress) {
        return ypr / 3.1415926535 * 180.0;
    } else {
        return ypr;
    }
}

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
    Eigen::Vector3d T = initial_pose.first.transpose()*(pose.second - initial_pose.second);

//    auto R = pose.first;
    T = Rcam.transpose() * T;

    auto eul = R2ypr(R);
    this->send_mapped_posedata(t, std::make_pair(eul, T));
}

void PoseRemapper::reset_center() {
    is_inited = false;
}
