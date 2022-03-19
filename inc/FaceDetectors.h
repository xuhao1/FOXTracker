#ifndef FACEDETECTOR_H
#define FACEDETECTOR_H
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <opencv2/opencv.hpp>
#include <FlightAgxSettings.h>
#include <fagx_datatype.h>
#include <onnxruntime_cxx_api.h>

class FaceDetector {
    cv::dnn::Net head_detector;
    cv::Rect2d last_roi;
public:
    FaceDetector() {
        printf("Path %s", settings->protoPath.c_str());
        head_detector = cv::dnn::readNetFromCaffe(settings->protoPath, settings->modelPath);
    }
    virtual ~FaceDetector() {

    }

    virtual cv::Rect2d detect(const cv::Mat & frame, cv::Rect2d predict_roi);
    virtual std::vector<cv::Rect2d> detect_objs(const cv::Mat & frame);
};



struct Landmarks {
    CvPts landmarks2d;
    CvPts3d landmarks3d;
    std::vector<float> confs;
};

class LandmarkDetector {
#ifndef Q_OS_ANDROID
    dlib::shape_predictor predictor;
#endif
    Ort::Env env;
    std::vector<Ort::Session*> sessions;
    float input_image[EMI_NN_MAX_INPUT*EMI_NN_MAX_INPUT*3] = {0};
    Ort::Value output_tensor_{nullptr};
    Ort::Value input_tensor_{nullptr};

    Ort::Value output_tensor14_{nullptr};
    Ort::Value input_tensor112_{nullptr};

    std::array<float, EMI_OUTPUT_CHANNELS*EMI_NN_MAX_OUTPUT*EMI_NN_MAX_OUTPUT> results_{};
    std::array<float, EMI_OUTPUT_CHANNELS*EMI_NN_MAX_OUTPUT*EMI_NN_MAX_OUTPUT> results_meidum_{};

    std::vector<const char*> input_node_names{"input"};
    std::vector<const char*> output_node_names{"output"};

    cv::Scalar mean_scaling;
    cv::Scalar std_scaling;

    std::vector<cv::Point3f> model_points_66, model_points_68;

public:
    LandmarkDetector();

    virtual ~LandmarkDetector() {}

    virtual Landmarks detect(cv::Mat & frame, cv::Rect roi);

    //This part of code is derived from AIRLegend's aitrack
    //See https://github.com/AIRLegend/aitrack/blob/master/AITracker/src/model.cpp
    std::pair<CvPts, std::vector<float>> proc_heatmaps(float* heatmaps, int x0, int y0, float scale_x, float scale_y);

    void normalize(cv::Mat& image);
    void transpose(float* from, float* dest, int dim_x = EMI_NN_MAX_INPUT, int dim_y=EMI_NN_MAX_INPUT);
};

cv::Rect crop_roi(cv::Rect2d predict_roi, const cv::Mat & _frame, double rate = 0.6);
Eigen::Vector3d eul_by_crop(cv::Rect2d roi);
#endif // FACEDETECTOR_H
