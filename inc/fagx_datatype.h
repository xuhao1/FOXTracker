#ifndef FAGXDATATYPE_H
#define FAGXDATATYPE_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#define M_PI (3.1415926535)
#include <QMetaType>
#include <QVariant>
#include <utils.h>

//Yaw Pitch Roll X Y Z
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pose6DoF;

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose_;

typedef std::vector<cv::Point2f> CvPts;
typedef std::vector<cv::Point3f> CvPts3d;

typedef Eigen::Matrix<double, 19, 19> Matrix19d;

typedef Eigen::Matrix<double, 19, 1> Vector19d;
typedef Eigen::Matrix<double, 13, 1> Vector13d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

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

    Pose(const Pose & a) {
        position = a.position;
        attitude = a.attitude;
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
