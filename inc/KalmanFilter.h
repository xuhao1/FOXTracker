#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <Eigen/Eigen>
#include <fagx_datatype.h>
#include <FlightAgxSettings.h>

using namespace Eigen;

template <int D>
class ExtendKalmanFilter12DOF {
protected:
    Matrix<double, D, 1> X; //qx qy qz qw x y z wx wy wz vx vy vz

     //x y z w
    Map<Quaterniond> q; //store as  qx, qy, qz, qw
    Map<Vector3d> T; //Translation
    Map<Vector3d> w; //angular velocity in world
    Map<Vector3d> v; //angular velocity in world

    Eigen::Matrix<double, D, D> P;

    Eigen::Matrix<double, 7, 7> R;
    Eigen::Matrix<double, D, D> Q;
    //[0-3] Q
    //[4-6] T
    //[7-10] W
    //[10 - 13] V
    bool initialized = false;
    double t0 = 0;
    double t_state = 0;

    virtual void update_cov(double _cov_Q) = 0;

    virtual Eigen::Matrix<double, 7, D> H0mat() {
        return Eigen::Matrix<double, 7, D>::Identity();
    }

    Vector7d h0() {
        //h for pose measurement
        return X.block<7, 1>(0, 0);
    }

    virtual Matrix<double, D, 1> f(double dt) = 0;
    virtual Eigen::Matrix<double, D, D> Fmat(double dt) = 0;

    ExtendKalmanFilter12DOF(): q(X.data()),
        T(X.data()+4),
        w(X.data()+7),
        v(X.data()+10)
    {

    }
    
public:

    Pose on_raw_pose_data(double t, Pose pose, int type);

    Pose get_realtime_pose() const {
        return Pose(T, q);
    }

    Eigen::Vector3d get_angular_velocity() const {
        return w;
    }

    Eigen::Vector3d get_linear_velocity() const {
        return v;
    }

    Eigen::Matrix<double, D, D> getP() const {
        return P;
    }

    Pose predict(double t);

    void predict_by_dt(double dt);
    
    virtual void reset() {
        initialized = false;
        P.setZero();
        X.setZero();
    }
};

class ExtendKalmanFilter12DOF_13: public ExtendKalmanFilter12DOF<13> {
protected:
    virtual void update_cov(double _cov_Q) override;
public:
    ExtendKalmanFilter12DOF_13():
        ExtendKalmanFilter12DOF<13>()
    {
        this->update_cov(settings->cov_Q_lm);
    }

    virtual Vector13d f(double dt) override;

    virtual Eigen::Matrix<double, 13, 13> Fmat(double dt) override;
};

class ExtendKalmanFilter12DOF_19 : public  ExtendKalmanFilter12DOF<19> {
protected:
    Map<Vector3d> wa; //angular acceleration
    Map<Vector3d> a; //acc

    virtual void update_cov(double _cov_Q) override;

public:

    ExtendKalmanFilter12DOF_19():
        ExtendKalmanFilter12DOF<19>(),
        wa(X.data() + 13),
        a(X.data()+16)
    {
        this->update_cov(settings->cov_Q_lm);
    }


    virtual Vector19d f(double dt) override;

    virtual Eigen::Matrix<double, 19, 19> Fmat(double dt) override;
};



template <int D>
Pose ExtendKalmanFilter12DOF<D>::on_raw_pose_data(double t, Pose pose, int type) {
    if(!initialized) {
        q = pose.att();
        T = pose.pos();
        w = Eigen::Vector3d::Zero();
        v = Eigen::Vector3d::Zero();
        P = Eigen::Matrix<double, D, D>::Identity();

        t0 = t;
        initialized = true;

        std::cout << "Initialized X[" << X.transpose() << "]^T" << std::endl;
        //std::cout << "Initialized P" << P.transpose() << std::endl;
        std::cout << "Initialized Q[" << q.coeffs().transpose() << "]T[" << T.transpose() << "]^T" << std::endl;
        return pose;
    }

    Quaterniond zq(pose.att());
    Eigen::Vector3d zT = pose.pos();
    Vector7d Z;
    Z.block<4, 1>(0, 0) = zq.coeffs();
    Z.block<3, 1>(4, 0) = zT;

    if(type == 0) {
        this->update_cov(settings->cov_Q_lm);
    } else {
        this->update_cov(settings->cov_Q_fsa);
    }
   //First we need to predict to this time
    predict(t);

    auto H = H0mat();
    //std::cout << "h0" << h0().transpose() << std::endl;
    //std::cout << "Z[" << Z.transpose() <<  "]^T" << std::endl;

    //Bug happen on auto on eigen
    Vector7d y = Z - h0();
    //std::cout << "y[" << y.transpose() <<  "]^T" << std::endl;
    Eigen::Matrix<double, 7, 7> S = H*P*H.transpose() + R;
    auto K = P*H.transpose()*S.inverse();

    //std::cout << "T0" << T.transpose() << "\nV0" << v.transpose() << std::endl;

    X = X + K*y;

    //std::cout << "y_after" << (Z - h0()).transpose() << std::endl;

    P = (Eigen::Matrix<double, D, D>::Identity() - K * H )*P;

    q.normalize();

    //std::cout << "Pose Measurement Z[" << Z.transpose();

    //std::cout << "Residual y[" << y.transpose() << "]^T" << std::endl;
    //    std::cout << "S\n" << S << std::endl;
    //    std::cout << "K\n" << K << std::endl;
    //std::cout << "X[" << X.transpose() << "]^T" << std::endl;
    //std::cout << "V[" << v.transpose() << "]^T" << std::endl;
    return get_realtime_pose();
}

template <int D>
Pose ExtendKalmanFilter12DOF<D>::predict(double t) {

    for (double t1 = this->t_state; t1 < t; t1 += settings->ekf_predict_dt) {
        double dt = settings->ekf_predict_dt;
        if (t - t1 < dt) {
            dt = t - t1;
        }
        this->predict_by_dt(dt);
    }
    this->t_state = t;
    return get_realtime_pose();
}

template <int D>
void ExtendKalmanFilter12DOF<D>::predict_by_dt(double dt){
    auto F = Fmat(dt);
    X = f(dt);
    P = F*P*F.transpose() + Q;
    q.normalize();
}
#endif // KALMANFILTER_H
