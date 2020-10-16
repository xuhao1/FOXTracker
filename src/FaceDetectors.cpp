#include "FaceDetectors.h"
#include <QDebug>
#include <dlib/opencv.h>
#include <opencv2/opencv.hpp>

using namespace cv;

inline cv::Rect2d rect2roi(dlib::rectangle ret) {
    return cv::Rect2d(ret.left(), ret.top(), ret.right() - ret.left(), ret.bottom() - ret.top());
}



LandmarkDetector::LandmarkDetector(std::string model_path):
    mean_scaling(0.485f, 0.456f, 0.406f),
    std_scaling(0.229f, 0.224f, 0.225f) {
    cv::divide(mean_scaling, std_scaling, mean_scaling);
    std_scaling *= 255.0f;
    if (settings->landmark_detect_method < 0) {
        qDebug() << "Will use dlib as landmark detector";
        dlib::deserialize(model_path.c_str()) >> predictor;
    } else {
        qDebug() << "Will use onnx as landmark detector";
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
//        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        printf("Using Onnxruntime C++ API\n");
        std::string model_path = settings->emilianavt_models[settings->landmark_detect_method];
        std::wstring unicode(model_path.begin(), model_path.end());
        session = new Ort::Session(env, unicode.c_str(), session_options);

        Ort::AllocatorWithDefaultOptions allocator;
        size_t num_input_nodes = session->GetInputCount();

        std::vector<int64_t> input_node_dims{1, 3, 224, 224};
        std::vector<int64_t> output_shape_{1, 2, 56, 56};
        printf("Number of inputs = %zu\n", num_input_nodes);
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        input_tensor_ = Ort::Value::CreateTensor<float>(memory_info,
                   input_image, 224*224*3, input_node_dims.data(), 4);
        output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, results_.data(), results_.size(), output_shape_.data(), output_shape_.size());
    }
}

//Part of code is derived from AIRLegend's AITracker.
//See also https://github.com/AIRLegend/aitrack/blob/master/AITracker/src/model.cpp
void LandmarkDetector::normalize(cv::Mat& image)
{
    cv::divide(image, std_scaling, image);

    /*float* ptr = image.ptr<float>();
    for (int channel = 0; channel < 3; channel++){
        for (int i = 0; i < 224 * 224; i++) {
            ptr[224*224*channel + i] /= std_scaling[channel];
        }
    }*/

    cv::subtract(image, mean_scaling, image);
}

void LandmarkDetector::transpose(float* from, float* dest, int dim_x, int dim_y)
{
    int stride = dim_x * dim_y;

    for (int c = 0; c < 3; c++)
    {
        for (int i = 0; i < dim_x * dim_y; i++)
        {
            dest[i + stride*c] = from[c + i*3];
        }
    }
}


CvPts LandmarkDetector::detect(cv::Mat & frame, cv::Rect roi) {
    CvPts pts;
    if (settings->landmark_detect_method < 0) {
        dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
        dlib::rectangle rect(roi.x, roi.y, roi.x + roi.width, roi.y + roi.height);
        dlib::full_object_detection shape = predictor(dlib_img, rect);
        for (unsigned i = 0; i < shape.num_parts(); i++) {
            auto p = shape.part(i);
            pts.push_back(cv::Point2d(p.x(), p.y()));
        }
    } else {
        cv::Rect2i roi_i = roi;
        if (roi_i.area() < MIN_ROI_AREA) {
            return pts;
        }
        cv::Mat face_crop = frame(roi_i);

        face_crop.convertTo(face_crop, CV_32F);
        cv::cvtColor(face_crop, face_crop, cv::COLOR_BGR2RGB);

        cv::resize(face_crop, face_crop, cv::Size(224, 224), NULL, NULL, cv::INTER_LINEAR);
        normalize(face_crop);
        transpose((float*)face_crop.data, input_image);

        //here we got the data
        auto output_tensors = session->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor_, 1, output_node_names.data(), 1);
        float* output_arr = output_tensors[0].GetTensorMutableData<float>();
        pts = proc_heatmaps(output_arr, roi_i.x, roi_i.y, ((double)roi_i.height)/224, ((double)roi_i.width)/224);
    }
    return pts;
}

float logit(float p)
{
    if (p >= 1.0)
        p = 0.99999;
    else if (p <= 0.0)
        p = 0.0000001;

    p = p / (1 - p);
    return log(p) / 16;
}

CvPts LandmarkDetector::proc_heatmaps(float* heatmaps, int x0, int y0, float scale_x, float scale_y)
{
    CvPts facical_landmarks;
    int heatmap_size = 784; //28 * 28;
    for (int landmark = 0; landmark < 66; landmark++)
    {
        int offset = heatmap_size * landmark;
        int argmax = -100;
        float maxval = -100;
        for (int i = 0; i < heatmap_size; i++)
        {
            if (heatmaps[offset + i] > maxval)
            {
                argmax = i;
                maxval = heatmaps[offset + i];
            }
        }

        int x = argmax / 28;
        int y = argmax % 28;


        float conf = heatmaps[offset + argmax];
        float res = 223;

        int off_x = floor(res * (logit(heatmaps[66 * heatmap_size + offset + argmax])) + 0.1);
        int off_y = floor(res * (logit(heatmaps[2 * 66 * heatmap_size + offset + argmax])) + 0.1);


        float lm_y = (float)y0 + (float)(scale_x * (res * (float(x) / 27.) + off_x));
        float lm_x = (float)x0 + (float)(scale_y * (res * (float(y) / 27.) + off_y));

        facical_landmarks.push_back(cv::Point2f(lm_x, lm_y));
    }
    return facical_landmarks;
}


std::vector<cv::Rect2d> FaceDetector::detect_objs(const cv::Mat & frame) {
    std::vector<cv::Rect2d> ret;
    if (settings->use_fsa) {
        cv::Mat _img;
        int _size_dnn = frame.cols*0.5;

        cv::resize(frame, _img, cv::Size(_size_dnn, _size_dnn));
        auto blob = cv::dnn::blobFromImage(_img, 1.0, cv::Size(_size_dnn, _size_dnn), cv::Scalar(104.0, 177.0, 123.0));
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
            auto roi = crop_roi(predict_roi, frame, 0.4);
            if (settings->use_fsa) {
//                roi = cv::Rect2d(0, 0, 640, 480);
            }

            if (last_roi.area() < MIN_ROI_AREA) {
                last_roi = roi;
            }

            if (roi.area()>MIN_ROI_AREA) {
                dets = detect_objs(frame(roi));
                for (auto & det: dets) {
                    det.x = det.x + roi.x;
                    det.y = det.y + roi.y;
                }
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
        } else {
            last_roi = cv::Rect2d(0, 0, frame.cols, frame.rows);
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


