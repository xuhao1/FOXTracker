#include "FaceDetectors.h"
#include <QDebug>
#include <dlib/opencv.h>

using namespace cv;

inline cv::Rect2d rect2roi(dlib::rectangle ret) {
    return cv::Rect2d(ret.left(), ret.top(), ret.right() - ret.left(), ret.bottom() - ret.top());
}

CvPts LandmarkDetector::detect(cv::Mat frame, cv::Rect roi) {
    CvPts pts;
    dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
    dlib::rectangle rect(roi.x, roi.y, roi.x + roi.width, roi.y + roi.height);
    dlib::full_object_detection shape = predictor(dlib_img, rect);
    for (unsigned i = 0; i < shape.num_parts(); i++) {
        auto p = shape.part(i);
        pts.push_back(cv::Point2d(p.x(), p.y()));
    }
    return pts;
}

std::vector<cv::Rect2d> FaceDetector::detect_objs(const cv::Mat & frame) {
    std::vector<cv::Rect2d> ret;
    if (settings->use_fsa) {
        qDebug() << "Using ssd detector";
        cv::Mat _img;
        cv::resize(frame, _img, cv::Size(300, 300));
        auto blob = cv::dnn::blobFromImage(_img, 1.0, cv::Size(300, 300), cv::Scalar(104.0, 177.0, 123.0));
        head_detector.setInput(blob);
        auto detection = head_detector.forward();

        Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());

        for (int i = 0; i < detectionMat.rows; i++)
        {
            float confidence = detectionMat.at<float>(i, 2);
            if (confidence > settings->SSDThreshold) {
                   int idx = static_cast<int>(detectionMat.at<float>(i, 1));
                   int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * frame.cols);
                   int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * frame.rows);
                   int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * frame.cols);
                   int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * frame.rows);

                   Rect object((int)xLeftBottom, (int)yLeftBottom,
                               (int)(xRightTop - xLeftBottom),
                               (int)(yRightTop - yLeftBottom));
                   ret.push_back(object);
            }
        }
    } else {

        dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
        std::vector<dlib::rectangle> dets = detector(dlib_img);
        for (auto _det : dets) {
            ret.push_back(rect2roi(_det));
        }
    }

    return ret;
}

cv::Rect2d FaceDetector::detect(const cv::Mat & frame, cv::Rect2d predict_roi) {
    try
    {
        if (frame.cols == 0) {
            return cv::Rect2d(0, 0, 0, 0);
        }

        std::vector<cv::Rect2d> dets;
        if(predict_roi.area() < 1) {
            //No previous Boundingbox
            dets = detect_objs(frame);
        } else {
            auto roi = crop_roi(predict_roi, frame, 0.6);
            if (settings->use_fsa) {
                roi = cv::Rect2d(0, 0, 640, 480);
            }

            dets = detect_objs(frame(roi));
            for (auto & det: dets) {
                det.x = det.x + roi.x;
                det.y = det.y + roi.y;
            }
        }

        if (dets.size() > 0) {
            auto det = dets[0];
            double overlap = (det & predict_roi).area();
            double area = det.area();

            for (auto _box : dets) {
                double _aera = _box.area();
                double _overlap = (_box & predict_roi).area();
//                qDebug() << "Overlap" << _overlap;
                if (_aera > area && (_overlap > overlap || overlap <= 0)) {
                    det = _box;
                    overlap = _overlap;
                }
            }

            return det;
        }

        return cv::Rect2d(0, 0, 0, 0);
    }
    catch (...) {
        qDebug() << "Face Detection failed...";
        return cv::Rect2d(0, 0, 0, 0);
    }

}

cv::Rect crop_roi(cv::Rect2d predict_roi, const cv::Mat & _frame, double rate) {
    int ex_x = predict_roi.width*rate;
    int ex_y = predict_roi.height*rate;

    int x = predict_roi.x - ex_x;
    int y = predict_roi.y - ex_y;

    if (x < 0) {
        x = 0;
    }
    if(y < 0) {
        y = 0;
    }

    int w = predict_roi.width + ex_x*2;
    int h = predict_roi.height + ex_y*2;
    if(x + w > _frame.cols) {
        w = _frame.cols - x - 1;
    }

    if(y + h > _frame.rows) {
        h = _frame.rows - y - 1;
    }

    return cv::Rect(x, y, w, h);
}

Eigen::Vector3d eul_by_crop(cv::Rect2d roi) {
    auto x = roi.x + roi.width/2;
    auto y = roi.y + roi.height/2;
    std::vector<cv::Point2f> pts{cv::Point2f(x, y)};
    std::vector<cv::Point2f> unpts;
    cv::undistortPoints(pts, unpts, settings->K, settings->D);

    double yaw = atan2(unpts[0].x, 1);
    double pitch = atan2(unpts[0].y, 1 / cos(yaw));
    return -Eigen::Vector3d(yaw, pitch, 0);
}


