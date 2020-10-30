#ifndef HEADPOSEDETECTOR_H
#define HEADPOSEDETECTOR_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Eigen>
#include <string>
#include <thread>
#include <QThread>
#include <QDateTime>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <QDebug>
#include <FlightAgxSettings.h>
#include <queue>
#include <mutex>
#include <fagx_datatype.h>
#include <KalmanFilter.h>
#include <inc/FSANet.h>
#include "inc/FaceDetectors.h"
#include <opencv2/tracking/tracker.hpp>
#include <PS3EYEDriver/src/ps3eye.h>
#include <PS3EYEDriver/src/ps3eye_capi.h>

class MainWindow;

class HeadPoseDetector;

class HeadPoseTrackDetectWorker: public QObject {
    Q_OBJECT

    HeadPoseDetector * hd;
    bool is_running = false;
public slots:
    void run();
    void stop();

public:
    HeadPoseTrackDetectWorker(HeadPoseDetector * _hd): hd(_hd) {}
};

struct HeadPoseDetectionResult {
    bool success = false;
    std::vector<Pose> detected_poses;

    //This is face surface ground speed
    Eigen::Vector3d face_ground_speed;

    HeadPoseDetectionResult(bool _success, std::vector<Pose> _poses, Eigen::Vector3d gspd):
        success(_success), detected_poses(_poses), face_ground_speed(gspd)
    {

    }

    HeadPoseDetectionResult():
        success(false), detected_poses(0), face_ground_speed(0, 0, 0) {

    }

    HeadPoseDetectionResult(const HeadPoseDetectionResult & hd) {
        success = hd.success;
        detected_poses = hd.detected_poses;
        face_ground_speed = hd.face_ground_speed;
    }
};

class HeadPoseDetector: public QObject {
    Q_OBJECT

    FaceDetector * fd = nullptr;
    LandmarkDetector * lmd = nullptr;

     ExtendKalmanFilter12DOF_13 ekf;
//    ExtendKalmanFilter12DOF_19 ekf;
    

    bool is_running = false;

    cv::VideoCapture cap;

    ps3eye_t * ps3cam = nullptr;

    std::mutex detect_mtx;

    std::mutex detect_frame_mtx;

    bool frame_pending_detect = false;
    cv::Mat frame_need_to_detect;
    cv::Rect2d roi_need_to_detect;

    std::thread detect_thread;

    QThread mainThread;
    QTimer * main_loop_timer;
    QThread detectThread;

    cv::Mat rvec_init, tvec_init;

    Eigen::Matrix3d Rface, Rcam;

    bool first_solve_pose = true;
    Eigen::Vector3d Tinit;

    std::vector<cv::Mat> frames;

    cv::Rect2d last_roi;
    int frame_count = 0;

    cv::Ptr<cv::Tracker> tracker;

    cv::Mat preview_image;

    cv::Mat last_clean_frame;
    std::vector<int> last_ids;

    CvPts last_landmark_pts;

    double t0;
    double dt = 0.03;
    double last_t = 0;

    bool paused = false;
    //dQ of FSA and PnP result
    Eigen::Quaterniond dq;

    bool inited = false;

    Pose P0;


    FSANet fsanet;

    std::pair<bool, Pose> solve_face_pose(CvPts landmarks, std::vector<cv::Point3f> landmarks_3d, cv::Mat & frame, Eigen::Vector3d fsa_ypr);
    void draw(cv::Mat & frame, cv::Rect2d roi, cv::Rect2d face_roi, cv::Rect2d fsa_roi, CvPts landmarks, Pose p, cv::Point3f track_spd);

    //In camera frame
    Eigen::Vector3d estimate_ground_speed_by_tracker(double z, cv::Rect2d roi, cv::Point3f track_spd);

    std::ofstream log;

public:
    MainWindow * main_window;

    HeadPoseDetector(): last_roi(0, 0, 0, 0) {
        cv::setNumThreads(1);
        log.open(settings->app_path + "/debug.txt", std::ofstream::out);
        is_running = false;
        fd = new FaceDetector;
        lmd = new LandmarkDetector();

        rvec_init = (cv::Mat_<double>(3,1) << 0.0, -0.5, -3);
        tvec_init = (cv::Mat_<double>(3,1) << 0.0, 0.0, -0.5);

        Rface << 0,  1, 0,
                    0,  0, -1, 
                    -1, 0, 0;

        Rcam << 0, 0, -1,
                -1, 0, 0,
                 0, 1, 0;



        connect(this, SIGNAL(start()),
                this, SLOT(start_slot()));

        connect(this, SIGNAL(stop()),
                this, SLOT(stop_slot()));
        this->moveToThread(&mainThread);
        mainThread.start();
        ps3eye_init();
    }

    //When using FSANet. First is PnP pose, next is FSANet pose
    HeadPoseDetectionResult detect_head_pose(cv::Mat frame, cv::Mat & _show, double t, double dt);

    void run_thread();

    void run_detect_thread();


public:
    cv::Mat & get_preview_image() {
        return preview_image;
    }

signals:
    void start();
    void stop();
    void on_detect_pose(double t, Pose_ pose);
    void on_detect_pose6d(double t, Pose6DoF pose);
    void on_detect_pose6d_raw(double t, Pose6DoF pose);
    void on_detect_twist(double t, Eigen::Vector3d w, Eigen::Vector3d v);
    void on_detect_P(double t, Matrix19d P);


private slots:
    void loop();
    void start_slot();
    void stop_slot();

public slots:
    void reset();
    void reset_detect();
    void pause();
public:
    template<typename T>
    void reduceVector(std::vector<T> &v, std::vector<uchar> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); i++)
            if (status[i])
                v[j++] = v[i];
        v.resize(j);
    }

};


#endif // HEADPOSEDETECTOR_H
