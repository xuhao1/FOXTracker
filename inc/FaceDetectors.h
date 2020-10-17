#ifndef FACEDETECTOR_H
#define FACEDETECTOR_H
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <opencv2/opencv.hpp>
#include <FlightAgxSettings.h>
#include <fagx_datatype.h>
#include <onnxruntime_cxx_api.h>

class FaceDetector {
    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
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




class LandmarkDetector {
    dlib::shape_predictor predictor;
    Ort::Env env;
    std::vector<Ort::Session*> sessions;
    float input_image[224*224*3] = {0};
    Ort::Value output_tensor_{nullptr};
    Ort::Value input_tensor_{nullptr};
    std::array<float, 1*2*56*56> results_{};
    std::vector<const char*> input_node_names{"input"};
    std::vector<const char*> output_node_names{"output"};

    cv::Scalar mean_scaling;
    cv::Scalar std_scaling;

    std::vector<cv::Point3f> model_points_66, model_points_68;

public:
    LandmarkDetector();

    virtual ~LandmarkDetector() {}

    virtual std::pair<CvPts,CvPts3d> detect(cv::Mat & frame, cv::Rect roi);

    //This part of code is derived from AIRLegend's aitrack
    //See https://github.com/AIRLegend/aitrack/blob/master/AITracker/src/model.cpp
    CvPts proc_heatmaps(float* heatmaps, int x0, int y0, float scale_x, float scale_y);

    void normalize(cv::Mat& image);
    void transpose(float* from, float* dest, int dim_x = 224, int dim_y=224);
};

cv::Rect crop_roi(cv::Rect2d predict_roi, const cv::Mat & _frame, double rate = 0.6);
Eigen::Vector3d eul_by_crop(cv::Rect2d roi);
#endif // FACEDETECTOR_H
