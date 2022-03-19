#include "stereo_bundle_adjustment.h"
#include "ceres/ceres.h"
#include "reprojection_error.h"

using namespace ceres;
// #define MONO_USE_LANDMARK_DEPTHS

StereoBundleAdjustment::StereoBundleAdjustment(const std::vector<Eigen::Vector3d> & _landmarks3d, 
        const std::vector<Eigen::Vector2d> & _landmarks_unit_1, 
        const std::vector<Eigen::Vector2d> & _landmarks_unit_2, 
        const std::vector<int> & _landmarks2d_1_index,
        const std::vector<int> & _landmarks2d_2_index,
        const std::vector<float> & _confs1,
        const std::vector<float> & _confs2,
        const Pose & _camera1, 
        const Pose & _camera2):
    landmarks3d(_landmarks3d),
    landmarks_unit_1(_landmarks_unit_1),
    landmarks_unit_2(_landmarks_unit_2),
    landmarks2d_1_index(_landmarks2d_1_index),
    landmarks2d_2_index(_landmarks2d_2_index),
    confs1(_confs1),
    confs2(_confs2),
    camera_pose_1(_camera1),
    camera_pose_2(_camera2)
{}


StereoBundleAdjustment::StereoBundleAdjustment(const std::vector<Eigen::Vector3d> & _landmarks3d, 
        const std::vector<Eigen::Vector2d> & _landmarks_unit_1, 
        const std::vector<int> & _landmarks2d_1_index,
        const std::vector<float> & _confs1,
        const Pose & _camera1):
    landmarks3d(_landmarks3d),
    landmarks_unit_1(_landmarks_unit_1),
    landmarks2d_1_index(_landmarks2d_1_index),
    confs1(_confs1),
    camera_pose_1(_camera1)
{}

StereoBundleAdjustment * StereoBundleAdjustment::create_from_cv_points(std::vector<cv::Point3f> lm3d, 
            std::vector<cv::Point2f> lm2d, const std::vector<float> confs, cv::Mat K, cv::Mat D) {
    std::vector<Eigen::Vector3d> landmarks3d;
    std::vector<Eigen::Vector2d> landmarks2d;
    std::vector<int> landmarks2d_1_index;
    std::vector<cv::Point2f> lm2d_un;
    cv::undistortPoints(lm2d, lm2d_un, K, D);

    for (size_t i = 0; i < lm3d.size(); i++) {
        landmarks2d_1_index.push_back(i);
        auto p3d = lm3d[i];
        landmarks3d.push_back(Eigen::Vector3d(p3d.x, p3d.y, p3d.z));
        auto p2d = lm2d_un[i];
        landmarks2d.push_back(Eigen::Vector2d(p2d.x, p2d.y));
    }

    return new StereoBundleAdjustment(landmarks3d, landmarks2d, landmarks2d_1_index, confs, Pose::Identity());
}

std::pair<Pose, Matrix6d> StereoBundleAdjustment::solve(const Pose & initial, bool est_extrinsic) {
    Problem problem;
    double pose_drone[7] = {0}; //x y z qx qy qz qw
    double cam_pose_2_inv[7] = {0};
    std::vector<double> depths1(landmarks3d.size());

    initial.to_vector(pose_drone);
    camera_pose_2.inverse().to_vector(cam_pose_2_inv);

    for (auto i = 0; i < depths1.size(); i ++) {
        depths1[i] = camera_pose_1.apply_inv_pose_to(initial*landmarks3d[i]).z();
    }

    for (auto index: landmarks2d_1_index) {
#ifdef MONO_USE_LANDMARK_DEPTHS
        auto cf = ReprojectionError_v2::Create(landmarks3d[index], landmarks_unit_1[index], 
                camera_pose_1, confs1[index]*focal_length/pixel_error);
        problem.AddResidualBlock(cf, nullptr, pose_drone, depths1.data() + index);
#else
        auto cf = ReprojectionError::Create(landmarks3d[index], landmarks_unit_1[index], 
                camera_pose_1, confs1[index]*focal_length/pixel_error);
        problem.AddResidualBlock(cf, nullptr, pose_drone);
#endif
    }
    
    if (!est_extrinsic) {
        for (auto index: landmarks2d_2_index) {
            auto cf = ReprojectionError::Create(landmarks3d[index], landmarks_unit_2[index], 
                    camera_pose_2, confs2[index]*focal_length/pixel_error);
            problem.AddResidualBlock(cf, nullptr, pose_drone);
        }
    } else {
        for (auto index: landmarks2d_2_index) {
            auto cf = ReprojectionExtrinsicError::Create(landmarks3d[index], landmarks_unit_2[index], confs2[index]*focal_length/pixel_error);
            problem.AddResidualBlock(cf, nullptr, pose_drone, cam_pose_2_inv);
        }

        Matrix6d sqrt_inf = Matrix6d::Identity();
        sqrt_inf.block<3, 3>(0, 0) = sqrt_inf.block<3, 3>(0, 0)*(57.3/1.0); //Set cov = 1deg**2
        sqrt_inf.block<3, 3>(3, 3) = sqrt_inf.block<3, 3>(3, 3)*(1/0.01); //Set cov = 1cm**2

        auto cf = CameraExtrinsicError::Create(camera_pose_2, sqrt_inf);
        problem.AddResidualBlock(cf, nullptr, cam_pose_2_inv);
    }

    ceres::LocalParameterization* pose_local_parameterization = new ceres::ProductParameterization (new ceres::IdentityParameterization(3), 
        new ceres::EigenQuaternionParameterization());

    problem.SetParameterization(pose_drone, pose_local_parameterization);
    if (est_extrinsic) {
        problem.SetParameterization(cam_pose_2_inv, pose_local_parameterization);
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::DENSE_QR;
    // options.minimizer_type = ceres::LINE_SEARCH;
    options.max_solver_time_in_seconds = 0.1;
    Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    est_drone_pose = Pose(pose_drone);
    // std::cout << summary.FullReport() << std::endl;
    std::cout << "Initial" << initial.tostr() << "Ret" << est_drone_pose.tostr() << std::endl;

    if (est_extrinsic) {
        cam_pose_2_est = Pose(cam_pose_2_inv).inverse();
        std::cout << "cam_pose_2_initial" << camera_pose_2.tostr() << "Ret" << cam_pose_2_est.tostr() << std::endl;
    } else {
        cam_pose_2_est = camera_pose_2;
    }

    ceres::Covariance::Options cov_options;
    ceres::Covariance covariance(cov_options);
    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back(std::make_pair(pose_drone, pose_drone));
    CHECK(covariance.Compute(covariance_blocks, &problem));

    Eigen::Matrix<double, 7, 7, RowMajor> cov_pose_drone;
    covariance.GetCovarianceBlock(pose_drone, pose_drone, cov_pose_drone.data());
    // std::cout << "covariance pose_drone\n" << cov_pose_drone << std::endl;

    Eigen::Matrix<double, 6, 6> cov6d = cov_pose_drone.block<6, 6>(0, 0);

    return std::make_pair(est_drone_pose, cov6d);
}

namespace ceres {
bool EigenQuaternionParameterization::Plus(const double* x_ptr,
                                           const double* delta,
                                           double* x_plus_delta_ptr) const {
  Eigen::Map<Eigen::Quaterniond> x_plus_delta(x_plus_delta_ptr);
  Eigen::Map<const Eigen::Quaterniond> x(x_ptr);

  const double norm_delta =
      sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
  if (norm_delta > 0.0) {
    const double sin_delta_by_delta = sin(norm_delta) / norm_delta;

    // Note, in the constructor w is first.
    Eigen::Quaterniond delta_q(cos(norm_delta),
                               sin_delta_by_delta * delta[0],
                               sin_delta_by_delta * delta[1],
                               sin_delta_by_delta * delta[2]);
    x_plus_delta = delta_q * x;
  } else {
    x_plus_delta = x;
  }

  return true;
}

bool EigenQuaternionParameterization::ComputeJacobian(const double* x,
                                                      double* jacobian) const {
  jacobian[0] =  x[3]; jacobian[1]  =  x[2]; jacobian[2]  = -x[1];  // NOLINT
  jacobian[3] = -x[2]; jacobian[4]  =  x[3]; jacobian[5]  =  x[0];  // NOLINT
  jacobian[6] =  x[1]; jacobian[7]  = -x[0]; jacobian[8]  =  x[3];  // NOLINT
  jacobian[9] = -x[0]; jacobian[10] = -x[1]; jacobian[11] = -x[2];  // NOLINT
  return true;
}
}