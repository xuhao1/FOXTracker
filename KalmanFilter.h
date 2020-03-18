#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <Eigen/Eigen>
#include <fagx_datatype.h>
#include <FlightAgxSettings.h>

using namespace Eigen;

typedef Matrix<double, 13, 1> Vector13d;
typedef Matrix<double, 7, 1> Vector7d;

class ExtendKalmanFilter12DOF {
    Vector13d X; //qx qy qz qw x y z wx wy wz vx vy vz

    //x y z w
    Map<Quaterniond> q; //store as  qx, qy, qz, qw
    Map<Vector3d> T; //Translation
    Map<Vector3d> w; //angular velocity in world
    Map<Vector3d> v; //angular velocity in world

    Eigen::Matrix<double, 13, 13> P;

    Eigen::Matrix<double, 7, 7> R;
    Eigen::Matrix<double, 13, 13> Q;
    //[0-3] Q
    //[4-6] T
    //[7-10] W
    //[10 - 13] V
    bool initialized = false;
    double t0 = 0;
    double t_state = 0;
public:
    ExtendKalmanFilter12DOF():
        q(X.data()),
        T(X.data()+4),
        w(X.data()+7),
        v(X.data()+10)
    {
        this->update_cov();
    }

    void update_cov() {
        R.setOnes();
        R = 0.001 * R;
        R.block<4, 4>(0, 0) = Eigen::Matrix4d::Ones() * settings->cov_Q;
        R.block<3, 3>(4, 4) = Eigen::Matrix3d::Ones() * settings->cov_T;


    }

    Pose get_realtime_pose() {
        return std::make_pair(q.toRotationMatrix(), T);
    }

    Pose predict(double t);
    void predict_by_dt(double dt);

    Pose on_raw_pose_data(double t, Pose pose);

    Vector7d h0() {
        //h for pose measurement
        return X.block<7, 1>(0, 0);
    }

    Vector13d f(double dt);

    Eigen::Matrix<double, 7, 13> H0mat() {
        return Eigen::Matrix<double, 7, 13>::Identity();
    }

    Eigen::Matrix<double, 13, 13> Fmat(double dt);

};

#endif // KALMANFILTER_H
