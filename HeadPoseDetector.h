#ifndef HEADPOSEDETECTOR_H
#define HEADPOSEDETECTOR_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <string>
#include <thread>

//R, T
typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose;

//Yaw Pitch Roll X Y Z
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pose6DoF;

typedef std::vector<cv::Point2f> CvPts;
class FaceDetector {
public:
    FaceDetector(std::string model) {

    }
    virtual cv::Rect2d detect(cv::Mat & frame);
};

class LandmarkDetector {
public:
    LandmarkDetector(std::string model) {

    }

    virtual CvPts detect(cv::Mat & frame, cv::Rect roi);
};

class HeadPoseDetector {
    FaceDetector * fd = nullptr;
    LandmarkDetector * lmd = nullptr;
    bool is_running = false;
    std::thread th;
public:
    HeadPoseDetector() {
        is_running = true;
        printf("Running HD Pose");
        th = std::thread([&]{

           this->run_thread();
        });
    }
    Pose6DoF detect_head_pose(cv::Mat frame);
    void run_thread();
    void reset();
    
    void start();
    void stop();
};
#endif // HEADPOSEDETECTOR_H
