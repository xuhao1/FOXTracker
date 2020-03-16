#ifndef FLIGHTAGXSETTINGS_H
#define FLIGHTAGXSETTINGS_H
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Eigen>
#include <chrono>

class FlightAgxSettings {
public:
    cv::Mat K;
    cv::Mat D;
    int detect_duration = 10;
    bool enable_preview = true;
    std::string model;
    std::string landmark_model;
    int camera_id = 0;
    bool enable_multithread_detect = true;
    FlightAgxSettings() {
        Eigen::Matrix3d K_eigen;
        Eigen::VectorXd D_eigen(5);
        K_eigen << 520.70925933,   0.,         319.58341522,
                  0.,         520.3492704,  231.99546224,
                  0.,           0.,           1.   ;
        D_eigen << 0.19808774, -0.68766424, -0.00180889,  0.0008008 ,  0.7539345;
        cv::eigen2cv(K_eigen, K);
        D_eigen = D_eigen.transpose();
        cv::eigen2cv(D_eigen, D);

        model = "C:\\Users\\plane\\Develop\\FlightAgentX\\assets\\model.txt";
        landmark_model = "C:\\Users\\plane\\Develop\\FlightAgentX\\assets\\shape_predictor_68_face_landmarks.dat";
    }
};


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

extern FlightAgxSettings * settings;
#endif // FLIGHTAGXSETTINGS_H
