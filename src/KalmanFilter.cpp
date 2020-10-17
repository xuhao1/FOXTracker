#include "KalmanFilter.h"
#include <qDebug>

Pose ExtendKalmanFilter12DOF::on_raw_pose_data(double t, Pose pose, int type) {
    if(!initialized) {
        q = pose.att();
        T = pose.pos();
        w = Eigen::Vector3d::Zero();
        v = Eigen::Vector3d::Zero();
        P = Eigen::Matrix<double, 19, 19>::Identity();

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

    P = (Eigen::Matrix<double, 19, 19>::Identity() - K * H )*P;

    //std::cout << "Pose Measurement Z[" << Z.transpose();

    //std::cout << "Residual y[" << y.transpose() << "]^T" << std::endl;
    //    std::cout << "S\n" << S << std::endl;
    //    std::cout << "K\n" << K << std::endl;
    //std::cout << "X[" << X.transpose() << "]^T" << std::endl;
    //std::cout << "V[" << v.transpose() << "]^T" << std::endl;
    return get_realtime_pose();
}

Pose ExtendKalmanFilter12DOF::predict(double t) {

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

void ExtendKalmanFilter12DOF::predict_by_dt(double dt){
    auto F = Fmat(dt);
    X = f(dt);
    P = F*P*F.transpose() + Q;
    q.normalize();
}

Eigen::Quaterniond w_dot_q(Eigen::Vector3d omg, Eigen::Quaterniond q) {
    Eigen::Quaterniond w(0, omg.x(), omg.y(), omg.z());
    return w*q;
}


Vector19d ExtendKalmanFilter12DOF::f(double dt) {
    Vector19d _X;
    _X.block<4, 1>(0, 0) = q.coeffs() + 0.5*w_dot_q(w, q).coeffs()*dt;
    _X.block<3, 1>(4, 0) = T + v*dt;
    _X.block<12, 1>(7, 0) = X.block<12, 1>(7, 0);

    return _X;
}

Matrix4d Dwq_by_q(Eigen::Vector3d w, Eigen::Quaterniond q) {
    Matrix4d J;
    J << 0, -w.z(), w.y(), w.x(),
         w.z(), 0, -w.x(), w.y(),
        -w.y(), w.x(), 0, w.z(),
        -w.x(), -w.y(), -w.z(), 0;
    return J;
}

Matrix<double, 4, 3> Dwq_by_w(Eigen::Vector3d omg, Eigen::Quaterniond q) {
    Matrix<double, 4, 3> J;
    J << q.w(), q.z(), -q.y(),
         -q.z(), q.w(), q.x(),
         q.y(), -q.x(), q.w(),
         -q.x(), -q.y(), -q.z();
    return J;
}

Eigen::Matrix<double, 19, 19> ExtendKalmanFilter12DOF::Fmat(double dt) {
    Eigen::Matrix<double, 19, 19> F;
    F.setZero();
    F.block<4, 4>(0, 0) = Matrix4d::Identity() + 0.5*Dwq_by_q(w, q)*dt;
    F.block<4, 3>(0, 7) = 0.5*dt*Dwq_by_w(w, q);
    F.block<3, 3>(4, 4) = Matrix3d::Identity();
    F.block<3, 3>(4, 10) = Matrix3d::Identity() * dt;
    F.block<12, 12>(7, 7) = Matrix<double, 12, 12>::Identity();
    //    std::cout << "F\n" << F << std::endl;
    return F;
}



void ExtendKalmanFilter12DOF::update_cov(double _cov_Q) {
//   R.setOnes();
//   R = 0.001 * R;
   R.setZero();
   R.block<4, 4>(0, 0) = Eigen::Matrix4d::Identity() * _cov_Q;
   R.block<3, 3>(4, 4) = Eigen::Matrix3d::Identity() * settings->cov_T;

   Q.setZero();
   double dt = settings->ekf_predict_dt;

   Matrix4d covQ = Eigen::Matrix4d::Identity() * settings->cov_W*pow(dt, 4)*0.25;
   Matrix3d covW = Eigen::Matrix3d::Identity() * settings->cov_W*pow(dt, 2);

   Matrix3d covWa = Eigen::Matrix3d::Identity() * settings->cov_W*dt;
   
   Matrix<double, 4, 3> covQW = Matrix<double, 4, 3>::Identity() * settings->cov_W*pow(dt, 3)*0.5;
   Matrix<double, 3, 4> covWQ = Matrix<double, 3, 4>::Identity() * settings->cov_W*pow(dt, 3)*0.5;


   Matrix3d covT = Eigen::Matrix3d::Identity() * settings->cov_V*pow(dt, 4)*0.25;
   Matrix3d covV = Eigen::Matrix3d::Identity() * settings->cov_V*pow(dt, 2)*0.5;
   Matrix3d covA = Eigen::Matrix3d::Identity() * settings->cov_V*dt;
   Matrix3d covTV = Eigen::Matrix3d::Identity() * settings->cov_V*pow(dt, 3);

   Q.block<4, 4>(0, 0) = covQ;
   Q.block<3, 3>(7, 7) = covW;
   Q.block<4, 3>(7, 0) = covQW;
   Q.block<3, 4>(0, 7) = covWQ;

   Q.block<3, 3>(4, 4) = covT;
   Q.block<3, 3>(10, 10) = covV;
   Q.block<3, 3>(10, 4) = covTV;
   Q.block<3, 3>(4, 10) = covTV;
   Q.block<3, 3>(13, 13) = covWa;
   Q.block<3, 3>(16, 16) = covA;

   //   std::cout << "Q\n" << Q << std::endl;
}

Eigen::Matrix<double, 7, 19> ExtendKalmanFilter12DOF::H0mat() {
    return Eigen::Matrix<double, 7, 19>::Identity();
}



void ExtendKalmanFilter12DOF::update_by_feature_pts(double t, std::pair<CvPts, CvPts> pts, std::vector<cv::Point3f> pts3d) {

}
