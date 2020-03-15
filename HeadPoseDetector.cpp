#include "HeadPoseDetector.h"
#include<dlib/opencv.h>
#include <QDebug>
#include <iostream>

using namespace cv;
using namespace std;
static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R, int degress = true)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    if (degress) {
        return ypr / M_PI * 180.0;
    } else {
        return ypr;
    }

}

void HeadPoseDetector::run_thread() {
    VideoCapture cap;
    if(!cap.open(0, cv::CAP_DSHOW))
        return;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    while(is_running) {
        Mat frame;
        cap >> frame;
        if( frame.empty() )
            break;
        auto ret = detect_head_pose(frame);

         if (ret.first) {
            auto pose = ret.second;
            //Send packet
            //Debug only
            double data[6] = {0};
            data[0] = - pose.second.x()*100;
            data[1] = pose.second.y()*100;
            data[2] = - pose.second.z()*100;

            data[3] = pose.first.x();
            data[4] = pose.first.y();
            data[5] = pose.first.z();
            udpsock->writeDatagram((char*)data, sizeof(double)*6, QHostAddress("127.0.0.1"), 4242);
        }

        if (enable_preview) {
            imshow("Preview", frame);
            if( waitKey(10) == 27 ) {
                is_running = false;
            }
        }
    }
}

CvPts LandmarkDetector::detect(cv::Mat & frame, cv::Rect roi) {
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

void HeadPoseDetector::start() {
    if (is_running) {
        return;
    }
    is_running = true;
    th = std::thread([&]{
        this->run_thread();
    });
}

void HeadPoseDetector::stop() {
    is_running = false;
    th.join();
}

void HeadPoseDetector::reset() {
    first_solve_pose = true;
    stop();
    start();
}

std::pair<bool, Pose6DoF> HeadPoseDetector::detect_head_pose(cv::Mat & frame) {
    cv::Rect2d roi = fd->detect(frame);
    CvPts landmarks = lmd->detect(frame, roi);
    auto ret = this->solve_face_pose(landmarks, frame);
    Eigen::Vector3d eul, T;
    if (ret.first) {
        auto pose = ret.second;
        eul = R2ypr(pose.first);
        // auto eul = pose.first.eulerAngles(2, 1, 0) * 180 / M_PI;
        //Eigen::Vector3d eul;
        //eul.x() = _eul.z();
        //eul.y() = _eul.y();
        //eul.z() = _eul.x();

        char rot[100] = {0};
        char translation[100] = {0};

        sprintf(rot, "Y %3.1f P %3.1f R %3.1f", eul.x(), eul.y(), eul.z());
        cv::putText(frame, rot, cv::Point(50, 70), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0,
                     Scalar(255, 0, 0), 2, 4);

        T = pose.second;

        if (enable_preview) {
            cv::rectangle(frame, cv::Point2d(roi.x, roi.y),
                cv::Point2d(roi.x + roi.width, roi.y + roi.height), cv::Scalar(255, 0, 0), 2);
            for (auto pt: landmarks) {
                cv::circle(frame, pt, 1, cv::Scalar(0, 255, 0), -1);
            }
        }
        return make_pair(true, make_pair(eul, T));
    }
    return make_pair(true, make_pair(eul, T));
}

std::pair<bool, Pose> HeadPoseDetector::solve_face_pose(CvPts landmarks, cv::Mat & frame) {
    bool success = false;
    auto rvec = rvec_init.clone();
    auto tvec = tvec_init.clone();
    success = cv::solvePnP(model_points_68, landmarks, K, D, rvec, tvec, true);

    if (success) {
        if (first_solve_pose) {
            cv::cv2eigen(tvec/1000.0, Tinit);
            first_solve_pose = false;
        }
    } else {
        qDebug() << "pnp Solve failed";
    }

    if (success && enable_preview) {
        cv::drawFrameAxes(frame, K, D, rvec, tvec, 30);
    }

    Eigen::Vector3d T;
    Eigen::Matrix3d R;
    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);

    cv::cv2eigen(tvec/1000.0, T);
    T = T - Tinit;
    cv::cv2eigen(Rcv, R);
//    std::cout << "R" << R <<std::endl<<std::endl;
    R = Rcam*R*Rface;
    return make_pair(success, make_pair(R, T));
}

cv::Rect2d FaceDetector::detect(cv::Mat & frame) {
    dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
    std::vector<dlib::rectangle> dets = detector(dlib_img);
    if (dets.size() > 0) {
        auto det = dets[0];
        double aera = (det.right()-det.left())*(det.bottom()-det.top());
        for (auto _box : dets) {
            double _aera = (_box.right()-_box.left())*(_box.bottom()-_box.top());
            if (_aera > aera) {
                det = _box;
            }
        }
        return cv::Rect2d(det.left(), det.top(), det.right() - det.left(), det.bottom() - det.top());
    }

    return cv::Rect2d(0, 0, 0, 0);
}
