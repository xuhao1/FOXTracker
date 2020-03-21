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
    dt = t - last_t;
    TicToc tic;
    auto ret = detect_head_pose(frame, t, dt);
    if (frame_count % 10 == 0)
        qDebug() << "detect_head_pose cost" << tic.toc();
    auto pose_raw = ret.second;
    auto pose = pose_raw;
    if (ret.first) {
        TicToc tic;
        auto Traw = pose_raw.second;
        auto Rraw = pose_raw.first;
        if(settings->use_ekf) {
            pose = ekf.on_raw_pose_data(t, pose_raw);
        } else {
            pose = pose_raw;
        }

        Rraw = Rcam*Rraw*Rface;
        Traw = Rcam*Traw;
        this->on_detect_pose6d_raw(t, make_pair(R2ypr(Rraw), Traw));
    }

    t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
    TicToc tic_ekf;

    if(settings->use_ekf) {
        pose = ekf.predict(t);
    }
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

    last_t = t;
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
//           cv::Rect2d roi = fd->detect(_frame, last_roi);
           cv::Rect2d roi = fd->detect(_frame, roi_need_to_detect);
           qDebug() << "Frontal face detector cost" << tic.toc() << "ms";
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
           for (auto & frame : frames) {
                bool success = tracker->update(frame, roi);
                if (!success) {
                    success_track = false;
                    break;
                }
           }
           int frame_size = frames.size();
           frames.clear();
           if(!success_track) {
//               qDebug() << "Tracker failed in detect thread queue size" << frame_size;
               frame_pending_detect = false;
               detect_mtx.unlock();
               continue;
           } else {
//               qDebug() << "Tracker OK in detect thread" << tic_retrack.toc() << "ms  queue size" << frame_size;
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
        last_landmark_pts.clear();
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

std::pair<CvPts, CvPts> calc_optical_flow(cv::Mat & prev_img, cv::Mat & cur_img, CvPts & prev_pts, CvPts predict_pts, std::vector<int> & ids, double dt = 0.03 ){
    vector<uchar> status;
    vector<float> err;
    TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 30, 0.01);
    CvPts cur_pts = predict_pts;
    CvPts pts_velocity;
    calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, Size(10,10), 2, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);

    reduceVector(prev_pts, status);
    reduceVector(cur_pts, status);
    reduceVector(ids, status);

    vector<uchar> reverse_status;
    vector<cv::Point2f> reverse_pts = prev_pts;
    cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(10, 10), 2, criteria, cv::OPTFLOW_USE_INITIAL_FLOW);
    for(size_t i = 0; i < reverse_status.size(); i++)
    {
       if(reverse_status[i] && cv::norm(prev_pts[i] - reverse_pts[i]) <= 0.5)
       {
           reverse_status[i] = 1;
       }
       else {
           reverse_status[i] = 0;
       }
    }

    reduceVector(prev_pts, reverse_status);
    reduceVector(cur_pts, reverse_status);
    reduceVector(ids, reverse_status);

    for (int i = 0; i < prev_pts.size(); i ++) {
        auto pt_vel = cur_pts[i] - prev_pts[i];
        pts_velocity.push_back(pt_vel/dt);
    }

    return make_pair(cur_pts, pts_velocity);
}

std::pair<bool, Pose> HeadPoseDetector::detect_head_pose(cv::Mat & frame, double t, double dt) {
    TicToc tic;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;

    cv::Rect2d roi;
    frame_count ++;
    auto frame_clean = frame.clone();
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
                roi_need_to_detect = last_roi;
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
            roi = fd->detect(frame, cv::Rect2d());
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
            if (new_add_pending_detect) {
                roi_need_to_detect  = roi;
            }
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

    if (last_landmark_pts.size() > 0) {
        // calculate optical flow
        TicToc tic;
        auto ret = calc_optical_flow(last_clean_frame, frame_clean, last_landmark_pts, landmarks, last_ids, dt);
        auto tracked_pts = ret.first;
        auto pts_velocity = ret.second;
        std::vector<cv::Point3f> pts3d;
        for (auto _id : last_ids) {
            pts3d.push_back(model_points_68[_id]);
        }
        ekf.update_by_feature_pts(t, ret, pts3d);
        if (settings->enable_preview) {
            for (int i = 0; i < tracked_pts.size(); i++) {
                cv::arrowedLine(frame, last_landmark_pts[i], tracked_pts[i], cv::Scalar(0, 0, 255), 1, 8, 0, 0.2);
            }
        }
    }

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

        last_landmark_pts = landmarks;

        last_clean_frame = frame_clean;

        for (int i = 0; i < 68; i++) {
            last_ids.push_back(i);
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


cv::Rect crop_roi(cv::Rect2d predict_roi, const cv::Mat & _frame) {
    int ex_x = predict_roi.width / 3;
    int ex_y = predict_roi.height / 3;

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

cv::Rect2d FaceDetector::detect(cv::Mat frame, cv::Rect2d predict_roi) {
    try {
        if (frame.cols == 0) {
            return cv::Rect2d(0, 0, 0, 0);
        }
        std::vector<dlib::rectangle> dets;
        if(predict_roi.area() < 1) {
            //No previous Boundingbox
            dlib::cv_image<dlib::rgb_pixel> dlib_img(frame);
            dets = detector(dlib_img);
        } else {
            auto roi = crop_roi(predict_roi, frame);
            dlib::cv_image<dlib::rgb_pixel> dlib_img(frame(roi));
            dets = detector(dlib_img);
            for (auto & det: dets) {
                det.left() = det.left() + roi.x;
                det.right() = det.right() + roi.x;
                det.top() = det.top() + roi.y;
                det.bottom() = det.bottom() + roi.y;
            }
//            cv::imshow("ROI to detect", frame(roi));
//            cv::waitKey(10);
        }

        if (dets.size() > 0) {
            auto det = dets[0];
            double overlap = (rect2roi(det) & predict_roi).area();
            double aera = (det.right()-det.left())*(det.bottom()-det.top());

            for (auto _box : dets) {
                double _aera = (_box.right()-_box.left())*(_box.bottom()-_box.top());
                double _overlap = (rect2roi(_box) & predict_roi).area();
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
        qDebug() << "Catch dlib detect failed...";
        return cv::Rect2d(0, 0, 0, 0);
    }

}


void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}


void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
