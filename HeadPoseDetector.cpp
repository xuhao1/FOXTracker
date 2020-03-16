#include "HeadPoseDetector.h"
#include<dlib/opencv.h>
#include <QDebug>
#include <iostream>
#include <windows.h>
#include <exception>

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
    if(!cap.open(settings->camera_id, cv::CAP_DSHOW))
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

        if (settings->enable_preview) {
            imshow("Preview", frame);
            if( waitKey(10) == 27 ) {
                is_running = false;
            }
        }
    }
}

CvPts LandmarkDetector::detect(cv::Mat frame, cv::Rect roi) {
    qDebug() << "Detecting landmark";
    CvPts pts;
    dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
    dlib::rectangle rect(roi.x, roi.y, roi.x + roi.width, roi.y + roi.height);
    dlib::full_object_detection shape = predictor(dlib_img, rect);
    for (unsigned i = 0; i < shape.num_parts(); i++) {
        auto p = shape.part(i);
        pts.push_back(cv::Point2d(p.x(), p.y()));
    }
    qDebug() << "Finish landmark";
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

    if( settings->enable_multithread_detect) {
        detect_thread = std::thread([&]{
            this->run_detect_thread();
        });
    }
}

void HeadPoseDetector::run_detect_thread() {
    while(is_running) {
        if (frame_pending_detect) {
           //Detect the roi
           qDebug() << "frame_pending_detect";
           TicToc tic;
           detect_frame_mtx.lock();
           cv::Mat _frame = frame_need_to_detect.clone();
           detect_frame_mtx.unlock();
           cv::Rect2d roi = fd->detect(_frame, last_roi);
           qDebug() << "Detect cost" << tic.toc() << "ms";
           //Track to now image

           if (roi.width < 1 && roi.height < 1) {
               frame_pending_detect = false;
               qDebug() << "Detect failed in thread";
               continue;
           } else {
               qDebug() << "Detect OK in thread";
           }


           cv::Ptr<cv::Tracker> tracker = TrackerMOSSE::create();
           detect_mtx.lock();

           tracker->init(_frame, roi);
           bool success_track = true;

           qDebug() << "Track " << frames.size() << "frames";
           TicToc tic_retrack;
//           for (auto & frame : frames) {
//                bool success = tracker->update(frame, roi);
//                if (!success) {
//                    success_track = false;
//                    break;
//                }
//           }

           if (frames.size() > 0) {
               success_track = tracker->update(frames.back(), roi);
           }

           frames.clear();
           if(!success_track) {
               qDebug() << "Tracker failed in detect thread";
               frame_pending_detect = false;
               detect_mtx.unlock();
               continue;
           } else {
               qDebug() << "Tracker OK in detect thread" << tic_retrack.toc() << "ms";
           }


           last_roi = roi;
           //delete this->tracker;
           this->tracker = tracker;
           frame_pending_detect = false;
           detect_mtx.unlock();

        } else {
            Sleep(10);
        }
    }
}

void HeadPoseDetector::stop() {
    is_running = false;
    th.join();
    detect_thread.join();
    //delete this->tracker;
}

void HeadPoseDetector::reset() {
    first_solve_pose = true;
    stop();
    start();
}

std::pair<bool, Pose6DoF> HeadPoseDetector::detect_head_pose(cv::Mat & frame) {
    TicToc tic;
    Eigen::Vector3d eul, T;
    cv::Rect2d roi;
    frame_count ++;
    if (first_solve_pose) {
        roi = fd->detect(frame, last_roi);
        if (roi.width > 1.0 && roi.height > 1.0) {
            tracker = cv::TrackerMOSSE::create();
            tracker->init(frame, roi);
        } else {
            return make_pair(false, make_pair(eul, T));
        }

    } else {
        bool new_add_pending_detect = false;
        bool in_thread_detected = false;
        if (frame_count % settings->detect_duration == 0) {
            if (settings->enable_multithread_detect && !frame_pending_detect) {
                frame_pending_detect = true;
                detect_frame_mtx.lock();
                frame.copyTo(frame_need_to_detect);
                detect_frame_mtx.unlock();

                new_add_pending_detect = true;
            }

            if (!settings->enable_multithread_detect) {
                roi = fd->detect(frame, last_roi);
                if (roi.width > 1.0 && roi.height > 1.0) {
                    in_thread_detected = true;
                    last_roi = roi;
                    tracker = cv::TrackerMOSSE::create();
                    tracker->init(frame, roi);
                }
            }
        }

        if (settings->enable_multithread_detect) {
            detect_mtx.lock();
        }
        if (!new_add_pending_detect && frame_pending_detect && settings->enable_multithread_detect) {
            //We add frames to help tracker
            frames.push_back(frame.clone());
        }

        roi = last_roi;
        TicToc tic;
        if (!in_thread_detected) {
            bool success = tracker->update(frame, roi);
            qDebug()<< "Using tracker" << tic.toc() << "ms status" << success;

            if (!success && !frame_pending_detect) {
                qDebug() << "Will detect in main thread";
                TicToc tic;
                roi = fd->detect(frame, last_roi);
                qDebug()<< "Tracker failed; Turn to detection" << tic.toc() << "ms";

                if (roi.width > 1.0 && roi.height > 1.0) {
                    last_roi = roi;
                    tracker = cv::TrackerMOSSE::create();
                    tracker->init(frame, roi);
                    success = true;
                }
            }

            if (success) {
                last_roi = roi;
                detect_mtx.unlock();
            } else {
                //std::cout << "Will unlock" << std::endl;
                detect_mtx.unlock();
                return make_pair(false, make_pair(eul, T));
            }
        } else {
            detect_mtx.unlock();
        }
    }

    if (roi.width < 1 || roi.height < 1) {
        return make_pair(false, make_pair(eul, T));
    }

    TicToc tic1;
    CvPts landmarks = lmd->detect(frame, roi);
    auto ret = this->solve_face_pose(landmarks, frame);

    if (ret.first) {
        auto pose = ret.second;
        eul = R2ypr(pose.first);

        char rot[100] = {0};
//        char translation[100] = {0};

        sprintf(rot, "Y %3.1f P %3.1f R %3.1f", eul.x(), eul.y(), eul.z());
        cv::putText(frame, rot, cv::Point(50, 70), cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0,
                     Scalar(255, 0, 0), 2, 4);

        T = pose.second;

        if (settings->enable_preview) {
            cv::rectangle(frame, cv::Point2d(roi.x, roi.y),
                cv::Point2d(roi.x + roi.width, roi.y + roi.height), cv::Scalar(255, 0, 0), 2);
            for (auto pt: landmarks) {
                cv::circle(frame, pt, 1, cv::Scalar(0, 255, 0), -1);
            }
        }
        return make_pair(true, make_pair(eul, T));
    }
    return make_pair(false, make_pair(eul, T));
}

std::pair<bool, Pose> HeadPoseDetector::solve_face_pose(CvPts landmarks, cv::Mat & frame) {
    bool success = false;
    auto rvec = rvec_init.clone();
    auto tvec = tvec_init.clone();
    success = cv::solvePnP(model_points_68, landmarks, settings->K, settings->D, rvec, tvec, true);

    if (success) {
        if (first_solve_pose) {
            cv::cv2eigen(tvec/1000.0, Tinit);
            first_solve_pose = false;
        }
    } else {
        qDebug() << "pnp Solve failed";
    }

    if (success && settings->enable_preview) {
        cv::drawFrameAxes(frame, settings->K, settings->D, rvec, tvec, 30);
    }

    Eigen::Vector3d T;
    Eigen::Matrix3d R;
    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);

    cv::cv2eigen(tvec/1000.0, T);
    T = T - Tinit;
    cv::cv2eigen(Rcv, R);
    R = Rcam*R*Rface;
    return make_pair(success, make_pair(R, T));
}

inline cv::Rect2d rect2roi(dlib::rectangle ret) {
    return cv::Rect2d(ret.left(), ret.top(), ret.right() - ret.left(), ret.bottom() - ret.top());
}

std::mutex dlib_mtx;

cv::Rect2d FaceDetector::detect(cv::Mat frame, cv::Rect2d last_roi) {
    try {
        qDebug() << "Tring detecting.... wait lock";
        if (!dlib_mtx.try_lock()) {
            return cv::Rect2d(0, 0, 0, 0);
        } else {
        }
        qDebug() << "CPY Frame"  << "\n";
        qDebug() << frame.cols<<"\n";
        qDebug() << "Detecting face..." << frame.cols <<"DEP"<< frame.depth() << "\n";
        if (frame.cols == 0) {
            dlib_mtx.unlock();
            return cv::Rect2d(0, 0, 0, 0);
        }
        dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
        std::vector<dlib::rectangle> dets = detector(dlib_img);
        qDebug() << "Finish dlib..." << dets.size();
        dlib_mtx.unlock();

        if (dets.size() > 0) {
            auto det = dets[0];
            double overlap = (rect2roi(det) & last_roi).area();
            double aera = (det.right()-det.left())*(det.bottom()-det.top());

            for (auto _box : dets) {
                double _aera = (_box.right()-_box.left())*(_box.bottom()-_box.top());
                double _overlap = (rect2roi(_box) & last_roi).area();
                qDebug() << "Overlap" << _overlap;
                if (_aera > aera && (_overlap > overlap || overlap <= 0)) {
                    det = _box;
                    overlap = _overlap;
                }
            }
            qDebug() << "Finish etecting face...";

            return rect2roi(det);
        }
        qDebug() << "Failed detecting face...";

        return cv::Rect2d(0, 0, 0, 0);
    } catch (...) {
        dlib_mtx.unlock();
        qDebug() << "Catch dlib detect failed...";
        return cv::Rect2d(0, 0, 0, 0);
    }

}
