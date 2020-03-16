#ifndef HEADPOSEDETECTOR_H
#define HEADPOSEDETECTOR_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <string>
#include <thread>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <QDebug>
#include <QtNetwork>
#include <FlightAgxSettings.h>
#include <queue>
#include <mutex>
#include <opencv2/tracking/tracker.hpp>

//R, T
typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose;

//Yaw Pitch Roll X Y Z
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pose6DoF;
typedef std::vector<cv::Point2f> CvPts;

extern std::mutex dlib_mtx;

class FaceDetector {
    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
public:
    FaceDetector() {

    }
    virtual ~FaceDetector() {

    }

    virtual cv::Rect2d detect(cv::Mat frame, cv::Rect2d last_roi);
};

class LandmarkDetector {
    dlib::shape_predictor predictor;
public:
    LandmarkDetector(std::string model_path) {
        dlib::deserialize(model_path.c_str()) >> predictor;
    }

    virtual ~LandmarkDetector() {}

    virtual CvPts detect(cv::Mat frame, cv::Rect roi);
};

class HeadPoseDetector {
    FaceDetector * fd = nullptr;
    LandmarkDetector * lmd = nullptr;
    bool is_running = false;
    std::thread th;

    std::mutex detect_mtx;

    std::mutex detect_frame_mtx;

    bool frame_pending_detect = false;
    cv::Mat frame_need_to_detect;

    std::thread detect_thread;

    std::pair<bool, Pose> solve_face_pose(CvPts landmarks, cv::Mat & frame);

    cv::Mat rvec_init, tvec_init;
    std::vector<cv::Point3f> model_points_68;

    Eigen::Matrix3d Rface, Rcam;

    bool first_solve_pose = true;
    Eigen::Vector3d Tinit;
    QUdpSocket * udpsock;

    std::vector<cv::Mat> frames;

    cv::Rect2d last_roi;
    int frame_count = 0;

    cv::Ptr<cv::Tracker> tracker;

public:
    HeadPoseDetector() {
        is_running = false;
        fd = new FaceDetector;
        lmd = new LandmarkDetector(settings->landmark_model);

        rvec_init = (cv::Mat_<double>(3,1) << 0.0, 0.0, -3.14392813);
        tvec_init = (cv::Mat_<double>(3,1) << 0.0, 0.0, -500);

        Rface << 0,  1, 0,
                    0,  0, -1, 
                    -1, 0, 0;

        Rcam << 0, 0, -1,
                -1, 0, 0,
                 0, 1, 0;

        std::ifstream model_file (settings->model);
        if (model_file.is_open())
        {
            double px, py, pz;
            while (!model_file.eof())
            {
                model_file >> px >> py >> pz;
                model_points_68.push_back(cv::Point3d(px, py, - pz));
            }
        }
        model_file.close();

        if(model_points_68.size() != 68) {
            qDebug() << "Model file error!";
            exit(-1);
        }

        udpsock = new QUdpSocket(nullptr);
        //udpsock->bind(QHostAddress::LocalHost, 4242);
    }

    std::pair<bool, Pose6DoF> detect_head_pose(cv::Mat & frame);
    void run_thread();
    void run_detect_thread();
    void reset();
    
    void start();
    void stop();
};
#endif // HEADPOSEDETECTOR_H
