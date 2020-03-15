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

//R, T
typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose;

//Yaw Pitch Roll X Y Z
typedef std::pair<Eigen::Vector3d, Eigen::Vector3d> Pose6DoF;

typedef std::vector<cv::Point2f> CvPts;
class FaceDetector {
    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
public:
    FaceDetector() {

    }
    virtual ~FaceDetector() {

    }

    virtual cv::Rect2d detect(cv::Mat & frame);
};

class LandmarkDetector {
    dlib::shape_predictor predictor;
public:
    LandmarkDetector(
        std::string model_path = "C:\\Users\\plane\\Develop\\FlightAgentX\\assets\\shape_predictor_68_face_landmarks.dat"
            ) {
        dlib::deserialize(model_path.c_str()) >> predictor;
    }

    virtual ~LandmarkDetector() {}

    virtual CvPts detect(cv::Mat & frame, cv::Rect roi);
};

class HeadPoseDetector {
    FaceDetector * fd = nullptr;
    LandmarkDetector * lmd = nullptr;
    bool is_running = false;
    std::thread th;
    bool enable_preview = true;
    std::pair<bool, Pose> solve_face_pose(CvPts landmarks, cv::Mat & frame);

    cv::Mat K, D;
    cv::Mat rvec_init, tvec_init;
    std::vector<cv::Point3f> model_points_68;

    Eigen::Matrix3d Rface, Rcam;

    bool first_solve_pose = true;
    Eigen::Vector3d Tinit;
    QUdpSocket * udpsock;
public:
    HeadPoseDetector(std::string model = "C:\\Users\\plane\\Develop\\FlightAgentX\\assets\\model.txt") {
        is_running = false;
        fd = new FaceDetector;
        lmd = new LandmarkDetector;
        Eigen::Matrix3d K_eigen;
        Eigen::VectorXd D_eigen(5);
        K_eigen << 520.70925933,   0.,         319.58341522,
                  0.,         520.3492704,  231.99546224,
                  0.,           0.,           1.   ;
//        K_eigen.transpose()
        D_eigen << 0.19808774, -0.68766424, -0.00180889,  0.0008008 ,  0.7539345;


        rvec_init = (cv::Mat_<double>(3,1) << 0.0, 0.0, -3.14392813);
        tvec_init = (cv::Mat_<double>(3,1) << 0.0, 0.0, -500);

        cv::eigen2cv(K_eigen, K);
        D_eigen = D_eigen.transpose();
        cv::eigen2cv(D_eigen, D);
        

        Rface << 0,  1, 0,
                    0,  0, -1, 
                    -1, 0, 0;

        Rcam << 0, 0, -1,
                -1, 0, 0,
                 0, 1, 0;

        //Rface = Rface.transpose();
        //Rcam = Rcam.transpose();

        std::ifstream model_file (model);
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
    void reset();
    
    void start();
    void stop();
};
#endif // HEADPOSEDETECTOR_H
