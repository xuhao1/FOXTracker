#ifndef FLIGHTAGXSETTINGS_H
#define FLIGHTAGXSETTINGS_H
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
#include <chrono>
#include <QApplication>
#include <QDebug>
#include <yaml-cpp/yaml.h>

#define MIN_ROI_AREA 10

#define DEG2RAD 3.1415926535/180
#define RAD2DEG 180/3.1415926535

//#define EMI_NN_SIZE (56)
//#define EMI_NN_OUTPUT_SIZE (7)
//#define EMI_FEATURE_NUM 30
//#define EMI_OUTPUT_CHANNELS (90)

#define EMI_NN_MAX_INPUT (224)
#define EMI_NN_MAX_OUTPUT (28)

#define EMI_NN_MEIDUM_INPUT (112)
#define EMI_NN_MEIDUM_OUTPUT (14)

#define EMI_FEATURE_NUM 66
#define EMI_OUTPUT_CHANNELS (198)

//#define EMI_NN_SIZE (224)
//#define EMI_NN_OUTPUT_SIZE (28)
//#define EMI_FEATURE_NUM 66
//#define EMI_OUTPUT_CHANNELS (198)

class FlightAgxSettings {
public:
    cv::Mat K;
    cv::Mat D;
    int detect_duration = 10;
    bool enable_preview = true;
    int camera_id = 0;
    bool enable_multithread_detect = true;
    int retrack_queue_size = 10;
    double fps = 30;
    double fsa_pnp_mixture_rate = 0.5;
    bool send_posedata_udp = true;
    int port = 4242;
    std::string udp_host = "127.0.0.1";

    std::string cfg_name = "/config.yaml";
    std::string trackir_path = "/assets/TrackIR.exe";
    std::string support_games_csv = "/assets/facetracknoir supported games.csv";
    std::string model_66 = "/assets/landmark_models/model_66.txt";
    std::string model_68 = "/assets/landmark_models/model_68.txt";

    std::string landmark_model = "/assets/landmark_models/shape_predictor_68_face_landmarks.dat";
    std::string fsanet_model = "/assets/fsanet_capsule.onnx";
    std::string protoPath ="/assets/face_detector/deploy.prototxt";
    std::string modelPath = "/assets/face_detector/res10_300x300_ssd_iter_140000.caffemodel";

    std::vector<std::string> emilianavt_models{
        "/assets/landmark_models/lm_modelV_opt.onnx",
        "/assets/landmark_models/lm_model0_opt.onnx",
        "/assets/landmark_models/lm_model1_opt.onnx",
        "/assets/landmark_models/lm_model2_opt.onnx",
        "/assets/landmark_models/lm_model3_opt.onnx"
    };

    int emi_nn_size = 224;
    int emi_nn_output_size = 28;


    std::string app_path;
    bool use_ft = false;
    bool use_npclient = false;
    double cov_Q_fsa = 0.006;
    double cov_Q_lm = 0.006;

    double cov_T = 0.01;

    double cov_V = 10.0;
    double cov_W = 2.0;

    double cov_gspd_planar = 0.01;

    double ekf_predict_dt = 0.01;

    bool use_ekf = false;

    bool use_fsa = true;

    double disp_duration = 30;

    int disp_max_series_size = 1000;

    double SSDThreshold = 0.5;

    double roi_filter_rate = 0.7;

    double cervical_face_model = -0.088;
    double cervical_face_model_x = 0.12;
    double cervical_face_model_y = 0.16;

    double pitch_offset_fsa_pnp = 11/180*3.1415926;

    //-1 dlib
    //0 network 0
    //network 1
    //2
    //3
    int landmark_detect_method = -1;

    int landmark_net_width = 224;
    double size = 0.09;

    std::vector<std::string> hotkey_joystick_names;
    std::vector<int> hotkey_joystick_buttons;

    void set_landmark_level(int landmark_level) {
        qDebug() << "Use landmark mode" << landmark_level;
        landmark_detect_method = landmark_level;
        if(landmark_level == 0) {
            emi_nn_size = 112;
            emi_nn_output_size = 14;
        } else {
            emi_nn_size = 224;
            emi_nn_output_size = 28;
        }
    }

    Eigen::Matrix3d Rcam;
    FlightAgxSettings(): hotkey_joystick_names(0),hotkey_joystick_buttons(0) {
        Eigen::Matrix3d K_eigen;
        Eigen::VectorXd D_eigen(5);

        //Surface camera
        // K_eigen << 520.70925933,   0.,         319.58341522,
        //           0.,         520.3492704,  231.99546224,
        //           0.,           0.,           1.   ;
        // D_eigen << 0.19808774, -0.68766424, -0.00180889,  0.0008008 ,  0.7539345;
        // D_eigen = D_eigen.transpose();

        // CL Eye
        //  K 
        // [[553.61456617   0.         308.32781287]
        // [  0.         556.75788726 252.73270154]
        // [  0.           0.           1.        ]] 
        // D 
        // [[-0.10055392  0.19422527  0.00414563 -0.00049292 -0.02306945]]
        K_eigen << 553.61456617,   0.,         308.32781287,
            0,         556.75788726, 252.73270154,
            0.,           0.,           1.;

        D_eigen <<  -0.10055392,  0.19422527,  0.00414563, -0.00049292, -0.02306945;

        cv::eigen2cv(K_eigen, K);
        cv::eigen2cv(D_eigen, D);

        app_path = QCoreApplication::applicationDirPath().toStdString();
        trackir_path = app_path + trackir_path;
        support_games_csv = app_path + support_games_csv;
        model_66 = app_path + model_66;
        model_68 = app_path + model_68;
        landmark_model = app_path + landmark_model;
        fsanet_model = app_path + fsanet_model;
        cfg_name = app_path + "/config.yaml";
        protoPath = app_path + protoPath;
        modelPath = app_path + modelPath;

        for (auto & st : emilianavt_models) {
            st = app_path + st;
        }

        qDebug() << "App run at" << app_path.c_str();

        Rcam << 0, 0, -1,
                -1, 0, 0,
                 0, 1, 0;

        load_from_config_yaml();
    }

    YAML::Node config;
    void load_from_config_yaml();
    void write_to_file();

    template<class T>
    void set_value(std::string k, T v, bool is_write_to_file=false) {
        config[k] = v;
        if(is_write_to_file) {
            write_to_file();
        }
    }
};




extern FlightAgxSettings * settings;
#endif // FLIGHTAGXSETTINGS_H
