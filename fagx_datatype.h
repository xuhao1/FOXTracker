#pragma once

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
//R, T
typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose;

//Yaw Pitch Roll X Y Z
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pose6DoF;
typedef std::vector<cv::Point2f> CvPts;
typedef Eigen::Matrix<double, 13, 13> Matrix13d;

static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R, int degress = true)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    if (degress) {
        return ypr / 3.1415926535 * 180.0;
    } else {
        return ypr;
    }
}

class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
