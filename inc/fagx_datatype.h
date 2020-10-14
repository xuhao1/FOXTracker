#ifndef FAGXDATATYPE_H
#define FAGXDATATYPE_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#define M_PI (3.1415926535)
#include <QMetaType>
#include <QVariant>

//Yaw Pitch Roll X Y Z
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pose6DoF;

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose_;

typedef std::vector<cv::Point2f> CvPts;
typedef Eigen::Matrix<double, 13, 13> Matrix13d;

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


class Pose {

    Eigen::Vector3d position = Eigen::Vector3d(0, 0, 0);
    Eigen::Quaterniond attitude = Eigen::Quaterniond(1, 0, 0, 0);

public:
    static Pose Identity() {
        return Pose();
    }



    Eigen::Vector3d ypr() const {
        return quat2eulers(attitude, false);
    }

    Pose(Eigen::Isometry3d trans) {
        position = trans.translation();
        attitude = trans.rotation();
    }

    Pose(Eigen::Vector3d pos, Eigen::Quaterniond att):
        position(pos), attitude(att)
    {
    }

    Pose(Eigen::Vector3d pos, Eigen::Vector3d att):
        position(pos)
    {
        attitude = Eigen::AngleAxisd(att(0), Eigen::Vector3d::UnitZ())
                   * Eigen::AngleAxisd(att(1), Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(att(2), Eigen::Vector3d::UnitX());
    }

    Pose(Eigen::Vector3d pos, Eigen::Matrix3d R):
        position(pos), attitude(R)
    {
    }

    Eigen::Isometry3d to_isometry() const {
        Eigen::Isometry3d a = Eigen::Translation3d(position) * attitude;
        return a;
    }

    friend Pose operator*(Pose a, Pose b) {
        Pose p;
        // p.position = a.attitude*(b.position+ a.position);
        p.position = a.attitude * b.position + a.position;
        p.attitude = a.attitude * b.attitude;

        return p;
    }

    
    friend Pose operator*(Eigen::Matrix3d R, Pose b) {
        Pose p;
        // p.position = a.attitude*(b.position+ a.position);
        //p.position = R * b.position;

        p.position = b.position;
        p.attitude = R * b.attitude;

        return p;
    }

    friend Eigen::Vector3d operator*(Pose a, Eigen::Vector3d point) {
        return a.attitude * point + a.position;
    }

    //A^-1B
    static Pose DeltaPose(const Pose &a, const Pose &b) {
        //Check this!!!
        Pose p;
        p.position = a.attitude.inverse() * (b.position - a.position);
        p.attitude = a.attitude.inverse() * b.attitude;
        return p;
    }

    void print() {
        auto _ypr = ypr();
        printf("T %3.3f %3.3f %3.3f RPY %3.1f %3.1f %3.1f\n",
               position.x(), position.y(), position.z(),
               _ypr.x() * 57.3,
               _ypr.y() * 57.3,
               _ypr.z() * 57.3);
    }

    Pose slerp(double rate, Pose a) {
        attitude = attitude.slerp(rate, a.att());
        position = position * (1 - rate) + a.pos() * rate;
        return Pose(position, attitude);
    }

    inline Eigen::Vector3d & pos() {
        return position;
    }

    inline Eigen::Quaterniond & att() {
        return attitude;
    }

    inline Eigen::Matrix3d R() const {
        return attitude.toRotationMatrix();
    }

    inline void set_att(Eigen::Quaterniond att) {
        attitude = att;
    }

    inline void set_pos(Eigen::Vector3d pos) {
        position = pos;
    }

    Pose() {
    }
};

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
