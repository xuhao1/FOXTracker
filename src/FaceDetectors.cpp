#include "FaceDetectors.h"
#include <QDebug>
#include <dlib/opencv.h>
#include <opencv2/opencv.hpp>
#include "cuda_provider_factory.h"
//#include "onnxruntime/core/session/dml_provider_factory.h"
#include "tensorrt_provider_factory.h"
using namespace cv;

inline cv::Rect2d rect2roi(dlib::rectangle ret) {
    return cv::Rect2d(ret.left(), ret.top(), ret.right() - ret.left(), ret.bottom() - ret.top());
}



LandmarkDetector::LandmarkDetector():
    mean_scaling(0.485f, 0.456f, 0.406f),
    std_scaling(0.229f, 0.224f, 0.225f) {
    cv::divide(mean_scaling, std_scaling, mean_scaling);
    std_scaling *= 255.0f;

    dlib::deserialize(settings->landmark_model.c_str()) >> predictor;

    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    if (settings->enable_gpu) {
        qDebug("Will use TensorRT to accelerate computing.....");
//        OrtSessionOptionsAppendExecutionProvider_Tensorrt(session_options, 0);
        //Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_CUDA(session_options, 0));
        //Ort::ThrowOnError(OrtSessionOptionsAppendExecutionProvider_Tensorrt(session_options, 0));
    }

    for (size_t i = 0; i < settings->emilianavt_models.size(); i ++) {
        std::string model_path = settings->emilianavt_models[i];
        std::wstring unicode(model_path.begin(), model_path.end());
        auto session = new Ort::Session(env, unicode.c_str(), session_options);
        sessions.push_back(session);
        qDebug("Added model from %s", model_path.c_str());
    }

    Ort::AllocatorWithDefaultOptions allocator;

    std::vector<int64_t> input_node_dims{1, 3, EMI_NN_MAX_INPUT, EMI_NN_MAX_INPUT};
    std::vector<int64_t> output_shape_{1, EMI_OUTPUT_CHANNELS, EMI_NN_MAX_OUTPUT, EMI_NN_MAX_OUTPUT};
    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    input_tensor_ = Ort::Value::CreateTensor<float>(memory_info,
               input_image, EMI_NN_MAX_INPUT*EMI_NN_MAX_INPUT*3, input_node_dims.data(), 4);
    output_tensor_ = Ort::Value::CreateTensor<float>(memory_info, results_.data(), results_.size(), output_shape_.data(), output_shape_.size());


    std::vector<int64_t> input_node_dims112{1, 3, EMI_NN_MEIDUM_INPUT, EMI_NN_MEIDUM_INPUT};
    std::vector<int64_t> output_shape14_{1, EMI_OUTPUT_CHANNELS, EMI_NN_MEIDUM_OUTPUT, EMI_NN_MEIDUM_OUTPUT};
    auto memory_info_112 = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    input_tensor112_ = Ort::Value::CreateTensor<float>(memory_info_112,
               input_image, EMI_NN_MEIDUM_INPUT*EMI_NN_MEIDUM_INPUT*3, input_node_dims112.data(), 4);
    //output_tensor14_ = Ort::Value::CreateTensor<float>(memory_info_112, results_meidum_.data(), EMI_NN_MEIDUM_OUTPUT*EMI_NN_MEIDUM_OUTPUT*3, output_shape14_.data(), output_shape14_.size());

    std::ifstream model_file (settings->model_68);
    if (model_file.is_open())
    {
        double px, py, pz;
        while (!model_file.eof())
        {
            model_file >> px >> py >> pz;
            model_points_68.push_back(cv::Point3d(px, -py, -(pz + settings->cervical_face_model)));
        }
    }

    std::ifstream model_file_66(settings->model_66);
    if (model_file_66.is_open())
    {
        double px, py, pz;
        while (!model_file_66.eof())
        {
            model_file_66 >> px >> py >> pz;
            model_points_66.push_back(cv::Point3d(px, -py, -(pz + settings->cervical_face_model)));
        }
    }

    qDebug("Load model with %ld pts", model_points_68.size());

    model_file.close();
    model_file_66.close();
}

//Part of code is derived from AIRLegend's AITracker.
//See also https://github.com/AIRLegend/aitrack/blob/master/AITracker/src/model.cpp
void LandmarkDetector::normalize(cv::Mat& image)
{
    cv::divide(image, std_scaling, image);
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


std::pair<CvPts,CvPts3d> LandmarkDetector::detect(cv::Mat & frame, cv::Rect roi) {
    CvPts pts;
    if (settings->landmark_detect_method < 0) {
#ifndef Q_OS_ANDROID
        dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
        dlib::rectangle rect(roi.x, roi.y, roi.x + roi.width, roi.y + roi.height);
        dlib::full_object_detection shape = predictor(dlib_img, rect);
        for (unsigned i = 0; i < shape.num_parts(); i++) {
            auto p = shape.part(i);
            pts.push_back(cv::Point2d(p.x(), p.y()));
        }
        return std::make_pair(pts, model_points_68);
#endif
    } else {
        cv::Rect2i roi_i = roi;
        if (roi_i.area() < MIN_ROI_AREA) {
            return std::make_pair(pts, model_points_66);
        }
        cv::Mat face_crop = frame(roi_i);

        cv::resize(face_crop, face_crop, cv::Size(settings->emi_nn_size, settings->emi_nn_size), NULL, NULL, cv::INTER_LINEAR);
        face_crop.convertTo(face_crop, CV_32F);
        cv::cvtColor(face_crop, face_crop, cv::COLOR_BGR2RGB);
        normalize(face_crop);
        transpose((float*)face_crop.data, input_image, settings->emi_nn_size, settings->emi_nn_size);
        //here we got the data
        assert(settings->landmark_detect_method < sessions.size() && "Landmark detect method must less than models");
        float* output_arr = nullptr;
        if (settings->landmark_detect_method <= 1) {
            auto output_tensors = sessions[settings->landmark_detect_method]->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor112_, 1, output_node_names.data(), 1);
            output_arr = output_tensors[0].GetTensorMutableData<float>();
        } else {
            auto output_tensors = sessions[settings->landmark_detect_method]->Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor_, 1, output_node_names.data(), 1);
            output_arr = output_tensors[0].GetTensorMutableData<float>();
        }
        pts = proc_heatmaps(output_arr, roi_i.x, roi_i.y, ((double)roi_i.height)/settings->emi_nn_size, ((double)roi_i.width)/settings->emi_nn_size);
        CvPts3d pts3d(model_points_66.begin(), model_points_66.begin() + EMI_FEATURE_NUM);
        return std::make_pair(pts, pts3d);
    }

    return std::make_pair(pts, model_points_66);
}

float logit(float p)
{
    if (p >= 1.0)
        p = 0.99999;
    else if (p <= 0.0)
        p = 0.0000001;

    p = p / (1 - p);

    if (settings->emi_nn_output_size == 7) {
        return log(p) / 8;
    } else {
        return log(p) / 16;
    }
}

CvPts LandmarkDetector::proc_heatmaps(float* heatmaps, int x0, int y0, float scale_x, float scale_y)
{
    CvPts facical_landmarks;
    int heatmap_size = settings->emi_nn_output_size*settings->emi_nn_output_size;
    for (int landmark = 0; landmark < EMI_FEATURE_NUM; landmark++)
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

        int x = argmax / settings->emi_nn_output_size;
        int y = argmax % settings->emi_nn_output_size;


        float conf = heatmaps[offset + argmax];
        float res = settings->emi_nn_size - 1;

        float off_x = res * (logit(heatmaps[EMI_FEATURE_NUM * heatmap_size + offset + argmax]));
        float off_y = res * (logit(heatmaps[2 * EMI_FEATURE_NUM * heatmap_size + offset + argmax]));

        float lm_y = (float)y0 + (float)(scale_x * (res * (float(x) / (settings->emi_nn_output_size-1)) + off_x));
        float lm_x = (float)x0 + (float)(scale_y * (res * (float(y) / (settings->emi_nn_output_size-1)) + off_y));

        facical_landmarks.push_back(cv::Point2f(lm_x, lm_y));
    }
    return facical_landmarks;
}


std::vector<cv::Rect2d> FaceDetector::detect_objs(const cv::Mat & frame) {
    std::vector<cv::Rect2d> ret;
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


