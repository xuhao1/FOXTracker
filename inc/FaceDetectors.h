#ifndef FACEDETECTOR_H
#define FACEDETECTOR_H
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <opencv2/opencv.hpp>
#include <inc/FlightAgxSettings.h>
#include <inc/fagx_datatype.h>

class FaceDetector {
    dlib::frontal_face_detector detector = dlib::get_frontal_face_detector();
    cv::dnn::Net head_detector;
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
public:
    LandmarkDetector(std::string model_path) {
        dlib::deserialize(model_path.c_str()) >> predictor;
    }

    virtual ~LandmarkDetector() {}

    virtual CvPts detect(cv::Mat frame, cv::Rect roi);
};

cv::Rect crop_roi(cv::Rect2d predict_roi, const cv::Mat & _frame, double rate = 0.6);
Eigen::Vector3d eul_by_crop(cv::Rect2d roi);
#endif // FACEDETECTOR_H
