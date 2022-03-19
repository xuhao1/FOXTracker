#pragma once
#include "Pose.h"
#include "opencv2/opencv.hpp"

class StereoBundleAdjustment {
    std::vector<Eigen::Vector3d> landmarks3d;
    std::vector<Eigen::Vector2d> landmarks_unit_1; 
    std::vector<Eigen::Vector2d> landmarks_unit_2;
    std::vector<int> landmarks2d_1_index;
    std::vector<int> landmarks2d_2_index;
    Pose camera_pose_1;
    Pose camera_pose_2;
    std::vector<float> confs1;
    std::vector<float> confs2;
    double focal_length = 300;
    double pixel_error = 6;

public:
    Pose est_drone_pose;
    Pose cam_pose_2_est;

    StereoBundleAdjustment(const std::vector<Eigen::Vector3d> & landmarks3d, 
        const std::vector<Eigen::Vector2d> & landmarks_unit_1, 
        const std::vector<Eigen::Vector2d> & landmarks_unit_2, 
        const std::vector<int> & landmarks2d_1_index,
        const std::vector<int> & landmarks2d_2_index,
        const std::vector<float> & confs1,
        const std::vector<float> & confs2,
        const Pose & camera1, 
        const Pose & camera2);
        
    StereoBundleAdjustment(const std::vector<Eigen::Vector3d> & landmarks3d, 
        const std::vector<Eigen::Vector2d> & landmarks_unit_1, 
        const std::vector<int> & landmarks2d_1_index,
        const std::vector<float> & confs1,
        const Pose & camera1);
    
    static StereoBundleAdjustment * create_from_cv_points(std::vector<cv::Point3f> lm3d, 
            std::vector<cv::Point2f> lm2d, const std::vector<float> confs, cv::Mat K, cv::Mat D);

    std::pair<Pose, Matrix6d> solve(const Pose & initial, bool est_extrinsic=false); //Return the pose of landmarks coordinates relative to camera
    std::vector<double*> landmarks;
};