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

    void reset() {
        initialized = false;
        P.setZero();
        X.setZero();
    }
    ExtendKalmanFilter12DOF():
        q(X.data()),
        T(X.data()+4),
        w(X.data()+7),
        v(X.data()+10)
    {
        this->update_cov();
    }

    void update_cov();

    Pose get_realtime_pose() const {
        return Pose(T, q);
    }

    Eigen::Vector3d get_angular_velocity() const {
        return w;
    }

    Eigen::Vector3d get_linear_velocity() const {
        return v;
    }

    Matrix13d getP() const {
        return P;
    }

    Pose predict(double t);
    void predict_by_dt(double dt);

    Pose on_raw_pose_data(double t, Pose pose, int type);

    Vector7d h0() {
        //h for pose measurement
        return X.block<7, 1>(0, 0);
    }

    Vector13d f(double dt);

    Eigen::Matrix<double, 7, 13> H0mat();

    Eigen::Matrix<double, 13, 13> Fmat(double dt);

    void update_by_feature_pts(double t, std::pair<CvPts, CvPts> pts, std::vector<cv::Point3f> pts3d);
};

#endif // KALMANFILTER_H
