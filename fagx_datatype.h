#pragma once

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
//R, T
typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose;

//Yaw Pitch Roll X Y Z
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pose6DoF;
typedef std::vector<cv::Point2f> CvPts;
