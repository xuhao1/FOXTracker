#include "HeadPoseDetector.h"
#include <dlib/opencv.h>
#include <QDebug>
#include <iostream>
#include <windows.h>
#include <exception>

using namespace cv;
using namespace std;

void HeadPoseTrackDetectWorker::run() {
    is_running = true;
//    hd->run_detect_thrad();
}

void HeadPoseTrackDetectWorker::stop() {
    is_running = false;

}

void HeadPoseDetector::loop() {
    Mat frame;
    cap >> frame;
    if( frame.empty() )
        return;
    double t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
    TicToc tic;
    auto ret = detect_head_pose(frame);
    if (frame_count % 10 == 0)
        qDebug() << "detect_head_pose cost" << tic.toc();
    auto pose_raw = ret.second;
    auto pose = pose_raw;
    if (ret.first) {
        TicToc tic;
        auto Traw = pose_raw.second;
        auto Rraw = pose_raw.first;

        pose = ekf.on_raw_pose_data(t, pose_raw);
        Rraw = Rcam*Rraw*Rface;
        Traw = Rcam*Traw;
        this->on_detect_pose6d_raw(t, make_pair(R2ypr(Rraw), Traw));
    }

    t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
    TicToc tic_ekf;
    pose = ekf.predict(t);
    this->on_detect_P(t, ekf.getP());

    auto R = pose.first;
    auto T = pose.second;
    R = Rcam*R*Rface;
    T = Rcam*T;

    //This pose is in world frame
    this->on_detect_pose(t, make_pair(R, T));
    this->on_detect_pose6d(t, make_pair(R2ypr(R), T));
    this->on_detect_twist(t, Rcam*ekf.get_angular_velocity(), Rcam*ekf.get_linear_velocity());

    if (settings->enable_preview) {
        frame.copyTo(preview_image);
    }
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


void HeadPoseDetector::run_detect_thread() {
    while(is_running) {
        if (frame_pending_detect) {
           TicToc tic;
           detect_frame_mtx.lock();
           cv::Mat _frame = frame_need_to_detect.clone();
           detect_frame_mtx.unlock();
           cv::Rect2d roi = fd->detect(_frame, last_roi);
           //qDebug() << "Detect cost" << tic.toc() << "ms";
           //Track to now image

           if (roi.width < 1 && roi.height < 1) {
               frame_pending_detect = false;
               qDebug() << "Detect failed in thread";
               continue;
           } else {
               //qDebug() << "Detect OK in thread";
           }


           cv::Ptr<cv::Tracker> tracker = TrackerMOSSE::create();
           detect_mtx.lock();

           tracker->init(_frame, roi);
           bool success_track = true;

           //qDebug() << "Track " << frames.size() << "frames";
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
//               qDebug() << "Tracker failed in detect thread";
               frame_pending_detect = false;
               detect_mtx.unlock();
               continue;
           } else {
//               qDebug() << "Tracker OK in detect thread" << tic_retrack.toc() << "ms";
           }


           last_roi = roi;
           this->tracker.release();
           this->tracker = tracker;
           frame_pending_detect = false;
           detect_mtx.unlock();

        } else {
            Sleep(10);
        }
    }
}


void HeadPoseDetector::start_slot() {
    t0 = QDateTime::currentMSecsSinceEpoch()/1000.0;
    if (is_running) {
        return;
    }

    is_running = true;

    if( settings->enable_multithread_detect) {
        detect_thread = std::thread([&]{
            this->run_detect_thread();
        });
    }

    this->run_thread();
}

void HeadPoseDetector::run_thread() {
    if(!cap.open(settings->camera_id, cv::CAP_DSHOW))
        return;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    main_loop_timer = new QTimer;
    main_loop_timer->moveToThread(&mainThread);
    connect(main_loop_timer, SIGNAL(timeout()), this, SLOT(loop()), Qt::DirectConnection);

    main_loop_timer->start(1/settings->fps);
}

void HeadPoseDetector::stop_slot() {
    if (is_running) {
        qDebug() << "Stoooop...";
        is_running = false;
        detect_thread.join();
        main_loop_timer->stop();
        ekf.reset();
        //wait a frame
        Sleep(30);
        cap.release();
    }
}

void HeadPoseDetector::reset() {
    first_solve_pose = true;
    stop();
    start();
}

std::pair<bool, Pose> HeadPoseDetector::detect_head_pose(cv::Mat & frame) {
    TicToc tic;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;

    cv::Rect2d roi;
    frame_count ++;
    if (first_solve_pose) {
        roi = fd->detect(frame, last_roi);
        if (roi.width > 1.0 && roi.height > 1.0) {
            tracker = cv::TrackerMOSSE::create();
            tracker->init(frame, roi);
        } else {
            return make_pair(false, make_pair(R, T));
        }

    } else {
        bool new_add_pending_detect = false;
        if (frame_count % settings->detect_duration == 0) {
            if (!frame_pending_detect) {
                frame_pending_detect = true;
                detect_frame_mtx.lock();
                frame.copyTo(frame_need_to_detect);
                detect_frame_mtx.unlock();

                new_add_pending_detect = true;
            }
        }

         detect_mtx.lock();
        if (!new_add_pending_detect && frame_pending_detect) {
            //We add frames to help tracker
            frames.push_back(frame.clone());
            while(frames.size() > settings->retrack_queue_size) {
                   frames.erase(frames.begin());
            }
        }

        roi = last_roi;
        TicToc tic;

        bool success = tracker->update(frame, roi);
        //qDebug()<< "Using tracker" << tic.toc() << "ms status" << success;

        if (!success && !frame_pending_detect) {
            qDebug() << "Will detect in main thread";
            TicToc tic;
            roi = fd->detect(frame, last_roi);
            qDebug()<< "Tracker failed; Turn to detection" << tic.toc() << "ms";

            if (roi.width > 1.0 && roi.height > 1.0) {
                last_roi = roi;
                tracker.release();
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
            return make_pair(false, make_pair(R, T));
        }

    }

    if (roi.width < 1 || roi.height < 1) {
        return make_pair(false, make_pair(R, T));
    }

    TicToc tic1;
    CvPts landmarks = lmd->detect(frame, roi);
    auto ret = this->solve_face_pose(landmarks, frame);

    if (ret.first) {
        auto pose = ret.second;
        char rot[100] = {0};
        T = pose.second;
        R = pose.first;

        if (settings->enable_preview) {
            cv::rectangle(frame, cv::Point2d(roi.x, roi.y),
                cv::Point2d(roi.x + roi.width, roi.y + roi.height), cv::Scalar(255, 0, 0), 2);
            for (auto pt: landmarks) {
                cv::circle(frame, pt, 1, cv::Scalar(0, 255, 0), -1);
            }
        }

        return make_pair(true, make_pair(R, T));
    }
    return make_pair(false, make_pair(R, T));
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
    cv::cv2eigen(Rcv, R);
    return make_pair(success, make_pair(R, T));
}

inline cv::Rect2d rect2roi(dlib::rectangle ret) {
    return cv::Rect2d(ret.left(), ret.top(), ret.right() - ret.left(), ret.bottom() - ret.top());
}

std::mutex dlib_mtx;

cv::Rect2d FaceDetector::detect(cv::Mat frame, cv::Rect2d last_roi) {
    try {
        if (!dlib_mtx.try_lock()) {
            return cv::Rect2d(0, 0, 0, 0);
        } else {
        }
        if (frame.cols == 0) {
            dlib_mtx.unlock();
            return cv::Rect2d(0, 0, 0, 0);
        }
        dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
        std::vector<dlib::rectangle> dets = detector(dlib_img);
        dlib_mtx.unlock();

        if (dets.size() > 0) {
            auto det = dets[0];
            double overlap = (rect2roi(det) & last_roi).area();
            double aera = (det.right()-det.left())*(det.bottom()-det.top());

            for (auto _box : dets) {
                double _aera = (_box.right()-_box.left())*(_box.bottom()-_box.top());
                double _overlap = (rect2roi(_box) & last_roi).area();
//                qDebug() << "Overlap" << _overlap;
                if (_aera > aera && (_overlap > overlap || overlap <= 0)) {
                    det = _box;
                    overlap = _overlap;
                }
            }
            //qDebug() << "Finish Detecting face...";

            return rect2roi(det);
        }
        //qDebug() << "Failed detecting face...";

        return cv::Rect2d(0, 0, 0, 0);
    } catch (...) {
        dlib_mtx.unlock();
        qDebug() << "Catch dlib detect failed...";
        return cv::Rect2d(0, 0, 0, 0);
    }

}
