#include "KalmanFilter.h"
#include <qDebug>

Eigen::Quaterniond w_dot_q(Eigen::Vector3d omg, Eigen::Quaterniond q) {
    Eigen::Quaterniond _w(0, omg.x(), omg.y(), omg.z());
    return _w*q;
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

Vector13d ExtendKalmanFilter12DOF_13::f(double dt) {
    Vector13d _X;
    _X.block<4, 1>(0, 0) = q.coeffs() + 0.5*w_dot_q(w, q).coeffs()*dt;
    _X.block<3, 1>(4, 0) = T + v*dt;
    _X.block<6, 1>(7, 0) = X.block<6, 1>(7, 0);

    return _X;
}

Eigen::Matrix<double, 13, 13> ExtendKalmanFilter12DOF_13::Fmat(double dt) {
    Eigen::Matrix<double, 13, 13> F;
    F.setZero();
    F.block<4, 4>(0, 0) = Matrix4d::Identity() + 0.5*Dwq_by_q(w, q)*dt;
    F.block<4, 3>(0, 7) = 0.5*dt*Dwq_by_w(w, q);
    F.block<3, 3>(4, 4) = Matrix3d::Identity();
    F.block<3, 3>(4, 10) = Matrix3d::Identity() * dt;
    F.block<6, 6>(7, 7) = Matrix<double, 6, 6>::Identity();
    return F;
}

void ExtendKalmanFilter12DOF_13::update_cov(double _cov_Q) {
    R.setZero();
   R.block<4, 4>(0, 0) = Eigen::Matrix4d::Identity() * _cov_Q;
   R.block<3, 3>(4, 4) = Eigen::Matrix3d::Identity() * settings->cov_T;

   Q.setZero();
   double dt = settings->ekf_predict_dt;

   Matrix4d covQ = Eigen::Matrix4d::Identity() * settings->cov_W*pow(dt, 4)*0.25;
   Matrix3d covW = Eigen::Matrix3d::Identity() * settings->cov_W*pow(dt, 2);
   Matrix<double, 4, 3> covQW = Matrix<double, 4, 3>::Identity() * settings->cov_W*pow(dt, 3)*0.5;
   Matrix<double, 3, 4> covWQ = Matrix<double, 3, 4>::Identity() * settings->cov_W*pow(dt, 3)*0.5;


   Matrix3d covT = Eigen::Matrix3d::Identity() * settings->cov_V*pow(dt, 4)*0.25;
   Matrix3d covV = Eigen::Matrix3d::Identity() * settings->cov_V*pow(dt, 2)*0.5;
   Matrix3d covTV = Eigen::Matrix3d::Identity() * settings->cov_V*pow(dt, 3);

   Q.block<4, 4>(0, 0) = covQ;
   Q.block<3, 3>(7, 7) = covW;
//    Q.block<4, 3>(7, 0) = covQW;
//    Q.block<3, 4>(0, 7) = covWQ;

   Q.block<3, 3>(4, 4) = covT;
   Q.block<3, 3>(10, 10) = covV;
   Q.block<3, 3>(10, 4) = covTV;
   Q.block<3, 3>(4, 10) = covTV;
}

void ExtendKalmanFilter12DOF_19::update_cov(double _cov_Q) {
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


Vector19d ExtendKalmanFilter12DOF_19::f(double dt) {
    Vector19d _X;
    _X.block<4, 1>(0, 0) = q.coeffs() + 0.5*w_dot_q(w, q).coeffs()*dt;
    _X.block<3, 1>(4, 0) = T + v*dt;
    _X.block<3, 1>(7, 0) = w + wa*dt;
    _X.block<3, 1>(10, 0) = v + a*dt;
    _X.block<6, 1>(13, 0) = X.block<6, 1>(13, 0);

    return _X;
}

Eigen::Matrix<double, 19, 19> ExtendKalmanFilter12DOF_19::Fmat(double dt) {
    Eigen::Matrix<double, 19, 19> F;
    F.setZero();
    F.block<4, 4>(0, 0) = Matrix4d::Identity() + 0.5*Dwq_by_q(w, q)*dt;
    F.block<4, 3>(0, 7) = 0.5*dt*Dwq_by_w(w, q);
    F.block<3, 3>(4, 4) = Matrix3d::Identity();
    F.block<3, 3>(4, 10) = Matrix3d::Identity() * dt;
    F.block<12, 12>(7, 7) = Matrix<double, 12, 12>::Identity();
    F.block<3, 3>(7, 13) = Matrix3d::Identity() * dt;
    F.block<3, 3>(10, 16) = Matrix3d::Identity() * dt;

    //    std::cout << "F\n" << F << std::endl;
    return F;
}