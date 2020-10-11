#include "HeadPoseDetector.h"
#include <dlib/opencv.h>
#include <QDebug>
#include <iostream>
#include <windows.h>
#include <exception>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;
cv::Rect crop_roi(cv::Rect2d predict_roi, const cv::Mat & _frame, double rate = 0.6);
Eigen::Vector3d eul_by_crop(double x, double y);

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

        //Use FSA Rotation
        if (!settings->use_fsa) {
            Rraw = Rcam*Rraw*Rface;
        }

        Traw = Rcam*Traw;
        auto ypr = R2ypr(Rraw);
        this->on_detect_pose6d_raw(t, make_pair(R2ypr(Rraw), Traw));

        inited = true;
    }

    t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
    TicToc tic_ekf;

    if(inited && settings->use_ekf) {
        pose = ekf.predict(t);
    }
    this->on_detect_P(t, ekf.getP());

    auto R = pose.first;
    auto T = pose.second;
    if (!settings->use_fsa) {
        R = Rcam*R*Rface;
    }
    T = Rcam*T;

    //This pose is in world frame
    if (ret.first || (settings->use_ekf && inited)) {
        this->on_detect_pose(t, make_pair(R, T));
        this->on_detect_pose6d(t, make_pair(R2ypr(R), T));
        this->on_detect_twist(t, Rcam*ekf.get_angular_velocity(), Rcam*ekf.get_linear_velocity());
    }

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


//           cv::Ptr<cv::Tracker> tracker = TrackerMOSSE::create();
           cv::Ptr<cv::Tracker> tracker = TrackerMOSSE::create();
           detect_mtx.lock();

           tracker->init(_frame, roi);
           bool success_track = true;

           qDebug() << "Track " << frames.size() << "frames";
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

    if( settings->enable_multithread_detect && settings->detect_method == 0) {
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


std::pair<bool, Pose> HeadPoseDetector::detect_head_pose(cv::Mat & frame, double t, double dt) {
    TicToc tic;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    CvPts landmarks;
    std::vector<cv::Point3f> landmarks_3d;
    cv::Mat frame_clean;
    cv::Rect2d roi;
    cv::Rect2d fsa_roi;

    Eigen::Vector3d fsa_ypr;

    if (settings->detect_method == 1) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners;
        TicToc tic;
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
        double dt = tic.toc();
        if (dt > 20) {
               qDebug() << "detectMarkers" << dt;
        }
        //Use id zero only now
        if (corners.size() > 0 ) {
            landmarks = corners[0];
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
            double size = 90;
            landmarks_3d = landmarks3D_ARMarker;
        }

    } else {
        frame_count ++;
        frame_clean = frame.clone();
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

        //Use FSA To Detect Rotation
        fsa_roi = crop_roi(roi, frame);
        TicToc fsa;
        if (fsa_roi.area() > 10*10) {
            fsa_ypr = fsanet.inference(frame(fsa_roi))
                    - eul_by_crop(fsa_roi.x + fsa_roi.width/2, fsa_roi.y + fsa_roi.height/2);
        }
        double dt_fsa = fsa.toc();
        TicToc tic1;
        landmarks = lmd->detect(frame, roi);
        qDebug() << "Landmark detector cost " << tic1.toc() << "FSA " << dt_fsa;
        landmarks_3d = model_points_68;
    }

    TicToc ticpnp;
    auto ret = this->solve_face_pose(landmarks, landmarks_3d, frame);
    if (ticpnp.toc() > 50) {
        qDebug() << "PnP " << tic.toc();
    }

    if (ret.first) {
        auto pose = ret.second;
        T = pose.second;
        R = pose.first;

        //Use FSA YPR Here
        if (settings->use_fsa) {
            R = Eigen::AngleAxisd(fsa_ypr(0), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(fsa_ypr(1), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(fsa_ypr(2), Eigen::Vector3d::UnitX());
        }

        if (settings->enable_preview) {
            cv::rectangle(frame, cv::Point2d(roi.x, roi.y),
                cv::Point2d(roi.x + roi.width, roi.y + roi.height), cv::Scalar(255, 0, 0), 2);
            cv::rectangle(frame, cv::Point2d(fsa_roi.x, fsa_roi.y),
                cv::Point2d(fsa_roi.x + fsa_roi.width, fsa_roi.y + fsa_roi.height), cv::Scalar(255, 255, 255), 2);
            for (auto pt: landmarks) {
                cv::circle(frame, pt, 1, cv::Scalar(0, 255, 0), -1);
            }
        }

        last_clean_frame = frame_clean;
        last_landmark_pts = landmarks;
        last_ids.clear();
        for (int i = 0; i < 68; i++) {
            last_ids.push_back(i);
        }


        return make_pair(true, make_pair(R, T));
    }
    return make_pair(false, make_pair(R, T));
}

std::pair<bool, Pose> HeadPoseDetector::solve_face_pose(CvPts landmarks, std::vector<cv::Point3f> landmarks_3d, cv::Mat & frame) {
    Eigen::Vector3d T;
    Eigen::Matrix3d R;

    if (landmarks.size() == 0) {
        return make_pair(false, make_pair(R, T));
    }
    bool success = false;
    auto rvec = rvec_init.clone();
    auto tvec = tvec_init.clone();

    TicToc tic;

    if (settings->detect_method == 1) {
        success = cv::solvePnP(landmarks_3d, landmarks, settings->K, settings->D, rvec, tvec, true, cv::SOLVEPNP_IPPE_SQUARE);
    } else {
        success = cv::solvePnP(landmarks_3d, landmarks, settings->K, settings->D, rvec, tvec, true);
    }

    if (tic.toc() > 30)
        qDebug() << "PnP Time" << tic.toc();

    if (success) {
        if (first_solve_pose) {
            cv::cv2eigen(tvec/1000.0, Tinit);
            first_solve_pose = false;
        }

        rvec_init = rvec;
        tvec_init = tvec;
    } else {
        qDebug() << "pnp Solve failed";
    }

//    if (success && settings->enable_preview) {
//        cv::drawFrameAxes(frame, settings->K, settings->D, rvec, tvec, 30);
//    }


    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);

    cv::cv2eigen(tvec/1000.0, T);
    cv::cv2eigen(Rcv, R);
    return make_pair(success, make_pair(R, T));
}

inline cv::Rect2d rect2roi(dlib::rectangle ret) {
    return cv::Rect2d(ret.left(), ret.top(), ret.right() - ret.left(), ret.bottom() - ret.top());
}

Eigen::Vector3d eul_by_crop(double x, double y) {
    std::vector<cv::Point2f> pts{cv::Point2f(x, y)};
    std::vector<cv::Point2f> unpts;
    cv::undistortPoints(pts, unpts, settings->K, settings->D);
//    qDebug() << "UNPTS [" <<  unpts[0].x << "," << unpts[0].y;

    double yaw = atan2(unpts[0].x, 1);
    double pitch = atan2(unpts[0].y, 1 / cos(yaw));
    return -Eigen::Vector3d(yaw, pitch, 0);
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

std::vector<cv::Rect2d> FaceDetector::detect_objs(const cv::Mat & frame) {
    std::vector<cv::Rect2d> ret;
    if (settings->use_fsa) {
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
