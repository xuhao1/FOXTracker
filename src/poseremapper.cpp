#include "poseremapper.h"
#include <utility>
#include <QDebug>
#include <FlightAgxSettings.h>

PoseRemapper::PoseRemapper(QObject *parent) : QObject(parent), _accela(&(settings->accela_s)), 
        _accela2(&(settings->accela_s))
{
    Rcam << 0, 0, -1,
           -1, 0, 0,
            0, 1, 0;


    pose_callback_timer = new QTimer;
    connect(pose_callback_timer, SIGNAL(timeout()), this, SLOT(pose_callback_loop()));
    pose_callback_timer->start(1000/POSE_OUTPUT_FREQ);

    t0 = QDateTime::currentMSecsSinceEpoch()/1000.0;
}

double double_constrain(double v, double vmin, double vmax) {
    if (v > vmax) {
        return vmax;
    }
    if (v < vmin) {
        return vmin;
    }

    return v;
}

double expo(const double &value, const double &e)
{
    double x = double_constrain(value, - 1, 1);
    double ec = double_constrain(e, 0, 1);
    return (1 - ec) * x + ec * x * x * x;
}

const double superexpo(const double &value, double e = 0.5, double g = 0.5)
{
    double x = double_constrain(value, - 1, 1);
    double gc = double_constrain(g, 0, 0.99);
    return expo(x, e) * (1 - gc) / (1 - fabsf(x) * gc);
}

double remap(double v, double input_bound, double output_bound, double _expo) {
    return expo(v / input_bound, _expo)*output_bound;
    //return v/input_bound*output_bound;
}

void PoseRemapper::pose_callback_loop() {
    if (settings->use_ft || settings->use_npclient) {
        double t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
        double dt = t - t_last;
        t_last = t;
        Eigen::Vector3d eul, T;
        if (settings->use_accela) {
            auto ret = _accela.filter(eul_last, T_last, dt);
            eul = ret.first;
            T = ret.second;
            if (settings->double_accela) {
                auto ret = _accela2.filter(eul, T, dt);
                eul = ret.first;
                T = ret.second;
            }
        }

        this->send_mapped_posedata(t, std::make_pair(eul, T));
    }
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

    if (settings->use_ft || settings->use_npclient) {
        T.x() = remap(T.x(), settings->inp_bound_trans.x(), settings->out_bound_trans.x(), settings->expo_trans.x());
        T.y() = remap(T.y(), settings->inp_bound_trans.y(), settings->out_bound_trans.y(), settings->expo_trans.y());
        T.z() = remap(T.z(), settings->inp_bound_trans.z(), settings->out_bound_trans.z(), settings->expo_trans.z());

        // std::cout << "Eul" << eul.x();
        eul.x() = remap(eul.x(), settings->inp_bound_eul.x(), settings->out_bound_eul.x(), settings->expo_eul.x());
        // std::cout<< "remapped" << eul.x() << "in bound" << settings->inp_bound_eul.x() << " out bound" << settings->out_bound_eul.x() << std::endl;

        eul.y() = remap(eul.y(), settings->inp_bound_eul.y(), settings->out_bound_eul.y(), settings->expo_eul.y());
        eul.z() = remap(eul.z(), settings->inp_bound_eul.z(), settings->out_bound_eul.z(), settings->expo_eul.z());
        eul_last = eul;
        T_last = T;
    } else {
        this->send_mapped_posedata(t, std::make_pair(eul, T));
    }

}

void PoseRemapper::reset_center() {
    is_inited = false;
}
