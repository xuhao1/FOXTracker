#ifndef UTILS_H
#define UTILS_H
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

template <typename T>
T inline wrap_angle(T angle) {
    while (angle > M_PI) {
        angle = angle - 2 * M_PI;
    }

    while (angle < -M_PI) {
        angle = angle + 2 * M_PI;
    }
    return angle;
}

inline cv::Rect2d mixture_roi(cv::Rect2d roia, cv::Rect2d roib, double rate) {
    cv::Rect2d ret_rect2d;
    ret_rect2d.x = roia.x * rate + roib.x*(1-rate);
    ret_rect2d.y = roia.y * rate + roib.y*(1-rate);
    ret_rect2d.width = roia.width * rate + roib.width*(1-rate);
    ret_rect2d.height = roia.height * rate + roib.height*(1-rate);

    return ret_rect2d;
}

inline Eigen::Vector3d quat2eulers(const Eigen::Quaterniond &quat, int degress = true) {
    Eigen::Vector3d ypr;
    ypr.z() = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()),
                    1 - 2 * (quat.x() * quat.x() + quat.y() * quat.y()));
    ypr.y() = asin(2 * (quat.w() * quat.y() - quat.z() * quat.x()));
    ypr.x() = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()),
                    1 - 2 * (quat.y() * quat.y() + quat.z() * quat.z()));
    if (degress) {
        return ypr / 3.1415926535 * 180.0;
    } else {
        return ypr;
    }
}

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

static double log_v(double v, double min, double max) {
    return min*exp(v*log(max/ min));
}

static double range_v(double v, double min, double max) {
    return v * (max-min) + min;
}

static double range_v_inv(double v, double min, double max) {
    return (v - min)/(max-min);
}


template <typename T>
static T clamp(T v, T min_, T max_)
{
       if (v > max_)
           return max_;
       if (v < min_)
           return min_;
       return v;
}

template <typename t>
constexpr int signum(const t& x)
{
    return x < t{0} ? -1 : 1;
}

#endif // UTILS_H
