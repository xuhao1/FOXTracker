#pragma once

#include "ceres/rotation.h"
#include "Pose.h"
#include "ceres/ceres.h"

struct ReprojectionError
{
    ReprojectionError(const Eigen::Vector3d &_landmark_drone, double _observed_x, double _observed_y, const Pose &_camera_pose, double _conf) : landmark_drone(_landmark_drone), observed_x(_observed_x), observed_y(_observed_y), conf(_conf)
    {
        Pose _cam_pose_inv = _camera_pose.inverse();
        quat_cam_inv = _cam_pose_inv.att();
        T_cam_inv = _cam_pose_inv.pos();
        // printf("Landmark %.1f %.1f %.1f observed %.3f %.3f, camera_pose_inv %s conf %.2f\n",
        //             _landmark_drone.x(), _landmark_drone.y(), _landmark_drone.z(), _observed_x, _observed_y,
        //             _camera_pose.inverse().tostr().c_str(),
        //             _conf);
    }

    template <typename T>
    bool operator()(const T *const drone_pose,
                    T *residuals) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(drone_pose);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(drone_pose + 3);
        Eigen::Matrix<T, 3, 1> landmark_body = q_a*landmark_drone.template cast<T>() + p_a;
        Eigen::Matrix<T, 3, 1> p = quat_cam_inv.template cast<T>()*landmark_body + T_cam_inv.template cast<T>();

        const T predicted_x = p(0) / p(2);
        const T predicted_y = p(1) / p(2);

        T _conf = (T)conf;
        residuals[0] = (predicted_x - observed_x) * _conf;
        residuals[1] = (predicted_y - observed_y) * _conf;

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &_landmark_drone, const Eigen::Vector2d &observed_undist, const Pose &_camera_pose, double conf)
    {
        double observed_x = observed_undist.x();
        double observed_y = observed_undist.y();

       return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7>(
            new ReprojectionError(_landmark_drone, observed_x, observed_y, _camera_pose, conf)));
    }

    double observed_x;
    double observed_y;
    double conf;
    Quaterniond quat_cam_inv;
    Vector3d T_cam_inv;
    Vector3d landmark_drone;
};


struct ReprojectionError_v2
{
    ReprojectionError_v2(const Eigen::Vector3d &_landmark_drone, Vector3d _observed_unit,
        const Pose &_camera_pose, double _conf) : landmark_drone(_landmark_drone), observed_unit(_observed_unit), conf(_conf)
    {
        Pose _cam_pose_inv = _camera_pose.inverse();
        quat_cam_inv = _cam_pose_inv.att();
        T_cam_inv = _cam_pose_inv.pos();
        // printf("Landmark %.1f %.1f %.1f observed %.3f %.3f, camera_pose_inv %s conf %.2f\n",
        //             _landmark_drone.x(), _landmark_drone.y(), _landmark_drone.z(), _observed_x, _observed_y,
        //             _camera_pose.inverse().tostr().c_str(),
        //             _conf);
    }

    template <typename T>
    bool operator()(const T *const drone_pose,
                    const T *const depth,
                    T *residuals) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(drone_pose);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(drone_pose + 3);
        Eigen::Matrix<T, 3, 1> landmark_body = q_a*landmark_drone.template cast<T>() + p_a;
        Eigen::Matrix<T, 3, 1> p = quat_cam_inv.template cast<T>()*landmark_body + T_cam_inv.template cast<T>();

        auto pt3d = observed_unit.template cast<T>() * depth[0];

        Eigen::Map<Eigen::Matrix<T, 3, 1>> res(residuals);
        T _conf = (T)conf;
        res = (pt3d - p)*_conf;

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &_landmark_drone, const Eigen::Vector2d &observed_undist, const Pose &_camera_pose, double conf)
    {
        Vector3d observed_unit_3d(observed_undist.x(), observed_undist.y(), 1.0);
        observed_unit_3d.normalize();

       return (new ceres::AutoDiffCostFunction<ReprojectionError_v2, 3, 7, 1>(
           new ReprojectionError_v2(_landmark_drone, observed_unit_3d, _camera_pose, conf)));
    }

    Vector3d observed_unit;
    double conf;
    Quaterniond quat_cam_inv;
    Vector3d T_cam_inv;
    Vector3d landmark_drone;
};

struct ReprojectionExtrinsicError
{
    ReprojectionExtrinsicError(const Eigen::Vector3d &_landmark_drone, double _observed_x, double _observed_y, double _conf) :
        landmark_drone(_landmark_drone), observed_x(_observed_x), observed_y(_observed_y), conf(_conf)
    {
        // printf("Landmark %.1f %.1f %.1f observed %.3f %.3f, conf %.2f\n",
        //             _landmark_drone.x(), _landmark_drone.y(), _landmark_drone.z(), _observed_x, _observed_y,
        //             _conf);
    }

    template <typename T>
    bool operator()(const T *const drone_pose, const T * const cam_pose_inv,
                    T *residuals) const
    {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(drone_pose);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(drone_pose + 3);

        Eigen::Map<const Eigen::Matrix<T, 3, 1>> T_cam_inv(cam_pose_inv);
        Eigen::Map<const Quaternion<T>> quat_cam_inv(cam_pose_inv+3);


        Eigen::Matrix<T, 3, 1> landmark_body = q_a*landmark_drone.template cast<T>() + p_a;
        Eigen::Matrix<T, 3, 1> p = quat_cam_inv*landmark_body + T_cam_inv.template cast<T>();

        const T predicted_x = p(0) / p(2);
        const T predicted_y = p(1) / p(2);

        T _conf = (T)conf;
        residuals[0] = (predicted_x - observed_x) * _conf;
        residuals[1] = (predicted_y - observed_y) * _conf;

        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d &_landmark_drone, const Eigen::Vector2d &observed_undist, double conf)
    {
        double observed_x = observed_undist.x();
        double observed_y = observed_undist.y();

        return (new ceres::AutoDiffCostFunction<ReprojectionExtrinsicError, 2, 7, 7>(
            new ReprojectionExtrinsicError(_landmark_drone, observed_x, observed_y, conf)));
    }

    double observed_x;
    double observed_y;
    double conf;

    Vector3d landmark_drone;
};

struct CameraExtrinsicError{
   CameraExtrinsicError(const Pose &pose, Matrix6d _sqrt_inf):
   sqrt_inf(_sqrt_inf)
   {
       quat_cam2_vins = pose.att();
       T_cam2_vins = pose.pos();
   }

   template <typename T>
   bool operator()( const T * const cam_pose_inv,
                   T *residuals_ptr) const
   {
       Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);

       Eigen::Map<const Quaternion<T>> quat_cam2_inv(cam_pose_inv+3);
       Eigen::Map<const Eigen::Matrix<T, 3, 1>> T_cam2_inv(cam_pose_inv);

       //CamPose*(Est_CamPose_inv) = Identity()
       //pcampose = qcampose*est_campose_inv_pos
       //res_quat = quat_cam2_inv*camera_pose
       //res_pos = qcampose*est_campose_inv_pos  - pcampose

       Eigen::Quaternion<T> delta_q =
           quat_cam2_vins.template cast<T>() * quat_cam2_inv;

       residuals.template block<3, 1>(0, 0) = T(2.0) * delta_q.vec();
       residuals.template block<3, 1>(3, 0) = quat_cam2_vins.template cast<T>() * T_cam2_inv - T_cam2_vins.template cast<T>();
       for (unsigned int i = 0; i < 6; i ++ ) {
           residuals(i) = residuals(i)*sqrt_inf(i, i);
       }

       return true;
   }

   static ceres::CostFunction *Create(const Pose & _camera_pose2, Matrix6d sqrt_inf)
   {
       return (new ceres::AutoDiffCostFunction<CameraExtrinsicError, 6, 7>(
           new CameraExtrinsicError(_camera_pose2, sqrt_inf)));
   }

   Eigen::Quaterniond quat_cam2_vins;
   Vector3d T_cam2_vins;
   Matrix6d sqrt_inf;
};
