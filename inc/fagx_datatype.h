#ifndef FAGXDATATYPE_H
#define FAGXDATATYPE_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <QMetaType>
#include <QVariant>
#include <utils.h>
#include "Pose.h"

//Yaw Pitch Roll X Y Z
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pose6DoF;

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose_;

typedef std::vector<cv::Point2f> CvPts;
typedef std::vector<cv::Point3f> CvPts3d;

typedef Eigen::Matrix<double, 19, 19> Matrix19d;

typedef Eigen::Matrix<double, 19, 1> Vector19d;
typedef Eigen::Matrix<double, 13, 1> Vector13d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

Q_DECLARE_METATYPE(Pose);

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

#endif
