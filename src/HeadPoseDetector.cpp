#include "HeadPoseDetector.h"
#include <QDebug>
#include <iostream>
#include <windows.h>
#include <exception>
#include <opencv2/aruco.hpp>
#include <QTimer>
#include <QMessageBox>
#include <mainwindow.h>
using namespace cv;
using namespace std;

cv::Ptr<cv::Tracker> create_tracker() {
    return cv::TrackerMedianFlow::create();
    //return cv::TrackerMOSSE::create();
}


void HeadPoseTrackDetectWorker::run() {
    is_running = true;
//    hd->run_detect_thrad();
}

void HeadPoseTrackDetectWorker::stop() {
    is_running = false;
}



void HeadPoseDetector::loop() {
    if (paused) {
        return;
    }

    TicToc tic_cap;
    Mat frame;
    if (ps3cam != nullptr) {
        frame = cv::Mat(480, 640, CV_8UC3);
        ps3eye_grab_frame(ps3cam, frame.data);
    } else {
        cap >> frame;
    }


    if( frame.empty() ) {
        qDebug() << "Empty frame";
        return;
    }
    double t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
    dt = t - last_t;
    last_t = t;
    TicToc tic;
    cv::Mat _show;
    auto ret = detect_head_pose(frame, _show, t, dt);
    auto poses_raw = ret.detected_poses;
    Pose pose;
    Pose pose_raw;

    if (ret.success) {
        if (poses_raw.size() == 1 || settings->landmark_detect_method >= 0) {
            pose_raw = poses_raw[0];
        } else {
            pose_raw = (poses_raw[1].slerp(settings->fsa_pnp_mixture_rate, poses_raw[0]));
        }

        TicToc tic;
        if(settings->use_ekf) {
            pose = ekf.update_raw_pose_data(t, poses_raw[0], 0);
            if (poses_raw.size() > 1) {
                pose = ekf.update_raw_pose_data(t, poses_raw[1], 1);
            }

            // pose = ekf.update_ground_speed(t, ret.face_ground_speed);

        } else {
            pose = pose_raw;
        }
        if (!inited) {
            P0 = pose;
            inited = true;
        }
    }

    auto q0_inv = P0.att().inverse();
    // q0_inv = Eigen::Quaterniond::Identity();
    this->on_detect_pose6d_raw(t, make_pair(R2ypr(q0_inv*pose_raw.R()), q0_inv*pose_raw.pos()));

    t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
    TicToc tic_ekf;

    if(inited && settings->use_ekf) {
        pose = ekf.predict(t);
    }
    
    auto R = pose.R();
    auto q = pose.att();
    auto T = pose.pos();
    auto l = fabs(settings->cervical_face_model_x);
    auto ly = fabs(settings->cervical_face_model_y);

    //This pose is in world frame
    if (ret.success || (settings->use_ekf && inited)) {
        this->on_detect_pose(t, make_pair(R*Rface, T));

        auto omg = ekf.get_angular_velocity();
        auto spd = ekf.get_linear_velocity();
        auto eul = quat2eulers(q0_inv*q);
        this->on_detect_pose6d(t, make_pair(eul, q0_inv*T));
        this->on_detect_twist(t, q0_inv*ekf.get_angular_velocity(), q0_inv*ekf.get_linear_velocity());
    
        auto _T = pose_raw.pos();
        // qDebug("Angular*l %f %f GSPD %f %f", -omg(1)*l, omg(0)*ly, ret.face_ground_speed(0), ret.face_ground_speed(1));
        log << -omg(1)*l << "," <<  omg(0)*ly << "," << ret.face_ground_speed(0) << "," << ret.face_ground_speed(1) << ","
            << spd(0) << "," << spd(1)  << "," << spd(2) << "," << _T(0) << "," << _T(1) << "," << _T(2) << ","
            << T(0) << "," << T(1) << "," << T(2) << std::endl;
    }

    if (settings->enable_preview) {
        _show.copyTo(preview_image);
    }
}


void HeadPoseDetector::run_detect_thread() {
    static int fc = 0;
    fc++;
    while(is_running) {
        if (frame_pending_detect) {
           TicToc tic;
           detect_frame_mtx.lock();
           cv::Mat _frame = frame_need_to_detect.clone();
           detect_frame_mtx.unlock();
           cv::Rect2d roi = fd->detect(_frame, roi_need_to_detect);

           if (fc%10 == 0) {
           qDebug() << "Frontal face detector cost" << tic.toc() << "ms";
           }
           //Track to now image

           if (roi.area() < MIN_ROI_AREA) {
               frame_pending_detect = false;
               roi_need_to_detect = cv::Rect2d(0, 0, _frame.cols, _frame.rows);
               qDebug() << "Detect failed in thread";
               Sleep(10);
               continue;
           }

           detect_mtx.lock();
           tracker.release();
           tracker = create_tracker();
           tracker->init(_frame, roi);
           bool success_track = true;
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
               qDebug() << "Tracker failed in detect thread queue size" << frame_size;
               frame_pending_detect = false;
               detect_mtx.unlock();
               continue;
           }
           last_roi = roi;
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

    if(settings->enable_multithread_detect) {
        detect_thread = std::thread([&]{
            this->run_detect_thread();
        });
    }

    this->run_thread();
}

void HeadPoseDetector::run_thread() {
    qDebug("%d PS3 EYE connected.", ps3eye_count_connected());
    if (ps3eye_count_connected() > 0) {
        ps3cam = ps3eye_open(settings->camera_id, 640, 480, settings->fps, PS3EYE_FORMAT_BGR);
        set_auto_expo(settings->enable_auto_expo);
        set_gain(settings->camera_gain);
        set_expo(settings->camera_expo);
    } else {
        if(!cap.open(settings->camera_id)) {
            qDebug() << "Not able to open camera" << settings->camera_id <<  "exiting";
            preview_image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
            char warn[100] = {0};
            sprintf(warn, "Camera ID %d Error. Change it in config menu!!!", settings->camera_id);
            cv::putText(preview_image, warn, cv::Point2f(20, 240), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            is_running = false;
            detect_thread.join();
            return;
        }
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(cv::CAP_PROP_FPS, settings->fps);
    }

    qDebug() << "Start Timer with fps of" << settings->fps + 10 << "Cap FPS"<< cap.get(cv::CAP_PROP_FPS);
    main_loop_timer = new QTimer;
    main_loop_timer->moveToThread(&mainThread);
    connect(main_loop_timer, SIGNAL(timeout()), this, SLOT(loop()));

    main_loop_timer->start(1.0/(settings->fps + 10)*1000);

}


void HeadPoseDetector::set_gain(double gain) {
    if (ps3cam != nullptr) {
        ps3eye_set_parameter(ps3cam, PS3EYE_GAIN, (int)(gain*63));
    }
}

void HeadPoseDetector::set_expo(double expo) {
    if (ps3cam != nullptr) {
        ps3eye_set_parameter(ps3cam, PS3EYE_EXPOSURE, (int)(expo*255));
    }
}


void HeadPoseDetector::set_auto_expo(bool enable_auto_expo) {
    if (ps3cam != nullptr) {
        ps3eye_set_parameter(ps3cam, PS3EYE_AUTO_GAIN, enable_auto_expo);
    }
}

void HeadPoseDetector::stop_slot() {
    if (is_running) {
        qDebug() << "Stop...";
        last_landmark_pts.clear();
        is_running = false;
        detect_thread.join();
        main_loop_timer->stop();
        ekf.reset();
        //wait a frame
        Sleep(30);

        if(cap.isOpened()) {
            cap.release();
        }

        if(ps3cam != nullptr) {
            ps3eye_close(ps3cam);
        }
    }
}

void HeadPoseDetector::pause() {
    paused = !paused;
}

void HeadPoseDetector::reset() {
    first_solve_pose = true;
    stop();
    start();
}

void HeadPoseDetector::reset_detect() {
    last_roi = cv::Rect2d(0, 0, 0, 0);
    roi_need_to_detect = cv::Rect2d(0, 0, 0, 0);
    first_solve_pose  = true;
}


HeadPoseDetectionResult HeadPoseDetector::detect_head_pose(cv::Mat frame, cv::Mat & _show, double t, double dt) {
    TicToc tic;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    CvPts landmarks;
    std::vector<cv::Point3f> landmarks_3d;
    cv::Mat frame_clean;
    cv::Rect2d roi;
    cv::Rect2d fsa_roi;
    cv::Rect2d face_roi;
    cv::Point3f track_spd(0, 0, 0);

    Eigen::Vector3d fsa_ypr;

    HeadPoseDetectionResult ret;

    frame_count ++;
    frame_clean = frame.clone();
    if (first_solve_pose) {
        roi = fd->detect(frame, last_roi);
        last_roi = roi;
        if (roi.area() > MIN_ROI_AREA) {
            tracker = create_tracker();
            tracker->init(frame, roi);
        } else {
            if (settings->enable_preview) {
                _show = frame.clone();
                draw(_show, roi, face_roi, fsa_roi, landmarks, Pose(T, R), track_spd);
            }
            return ret;
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
        if (success) {
            track_spd.x = (roi.x + roi.width/2 - last_roi.x - last_roi.width/2)/dt;
            track_spd.y = (roi.y + roi.height/2 - last_roi.y - last_roi.height/2)/dt;
            track_spd.z = (1/roi.area() - 1/last_roi.area());
        }

        if (!success && !frame_pending_detect) {
            qDebug() << "Will detect in main thread";
            TicToc tic;
            roi = fd->detect(frame, cv::Rect2d());
            qDebug()<< "Tracker failed; Turn to detection" << tic.toc() << "ms";

            if (roi.area() > MIN_ROI_AREA) {
                last_roi = roi;
                tracker.release();
                tracker = create_tracker();
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
            if (settings->enable_preview) {
                _show = frame.clone();
                draw(_show, roi, face_roi, fsa_roi, landmarks, Pose(T, R), track_spd);
            }
            return ret;
        }

    }

    if (roi.area() < MIN_ROI_AREA) {
        if (settings->enable_preview) {
            _show = frame.clone();
            draw(_show, roi, face_roi, fsa_roi, landmarks, Pose(T, R), track_spd);
        }
        return ret;
    }

    double detect_track_time = tic.toc();
    face_roi = roi;
    fsa_roi = crop_roi(roi, frame, 0.2);
    static Rect2d fsa_roi_last;
    if (fsa_roi_last.area() < MIN_ROI_AREA) {
        fsa_roi_last = fsa_roi;
    } else {
        fsa_roi = fsa_roi_last = mixture_roi(fsa_roi_last, fsa_roi, settings->roi_filter_rate);
    }

    //Use FSA To Detect Rotation
    TicToc fsa;
    if(settings->use_fsa && settings->landmark_detect_method < 0) {

        if (fsa_roi.area() > MIN_ROI_AREA) {
            auto fsa_ypr_raw = fsanet.inference(frame(fsa_roi));
            fsa_ypr = fsa_ypr_raw - eul_by_crop(roi);
            if (settings->landmark_detect_method < 0) {
                if (fsa_ypr_raw(0) > 0) {
                    face_roi.x = face_roi.x - fsa_ypr_raw(0)*face_roi.width*0.3;
                }

                face_roi.width = face_roi.width + fabs(fsa_ypr_raw(0))*face_roi.width*0.3;

                if (fsa_ypr_raw(1) < 0) {
                    face_roi.height = face_roi.height - fabs(fsa_ypr_raw(1))*face_roi.height*0.1;
                }
            }
        }
    }
    double dt_fsa = fsa.toc();

    TicToc tic1;
    if (settings->landmark_detect_method < 0) {
        auto lmd_ret = lmd->detect(frame, face_roi);
        landmarks = lmd_ret.first;
        landmarks_3d = lmd_ret.second;
    } else {
        auto lmd_ret = lmd->detect(frame, fsa_roi);
        landmarks = lmd_ret.first;
        landmarks_3d = lmd_ret.second;
    }

    if (landmarks.size() != landmarks_3d.size()) {
        qDebug("Landmark detection failed. 2D pts %d 3D pts %d", landmarks.size(), landmarks_3d.size());
        if (settings->enable_preview) {
            _show = frame.clone();
            draw(_show, roi, face_roi, fsa_roi, landmarks, Pose(T, R), track_spd);
        }
        return ret;
    }

    if (frame_count % ((int)settings->fps) == 0)
        qDebug() << "Detect track" << detect_track_time <<  "Landmark detector cost " << tic1.toc() << "FSA " << dt_fsa;

    TicToc ticpnp;
    auto _ret = this->solve_face_pose(landmarks, landmarks_3d, frame, fsa_ypr);

    //Estimate Planar speed of face with tracker
    ret.face_ground_speed = estimate_ground_speed_by_tracker(_ret.second.pos().z(), roi, track_spd);

    if (_ret.first) {
        auto pose = _ret.second;
        T = pose.pos();

        //Pose 0 is PnP pose
        //Pose 1 is FSA Pose
        double offset = settings->pitch_offset_fsa_pnp;
        if(settings->landmark_detect_method < 0) {
            // offset = 0;
        }

        //pose.att() = pose.att() * Eigen::AngleAxisd(-offset, Eigen::Vector3d::UnitX());

        ret.detected_poses.push_back(pose);
        ret.success = true;

        //Use FSA YPR Here
        if (settings->use_fsa) {
             R = Rcam.transpose() * Eigen::AngleAxisd(fsa_ypr(0), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(fsa_ypr(1) + offset, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(-fsa_ypr(2), Eigen::Vector3d::UnitX())*Rface.transpose();
            Eigen::Quaterniond qR(R);
            ret.detected_poses.push_back(Pose(T, qR));

            //The mismatch here is a bug of the new model
            dq = pose.att()*qR.inverse();
        }

        if (settings->enable_preview) {
            _show = frame.clone();
            draw(_show, roi, face_roi, fsa_roi, landmarks, Pose(T, R), track_spd);
        }

        last_clean_frame = frame_clean;
        last_landmark_pts = landmarks;
        last_ids.clear();
        for (int i = 0; i < 68; i++) {
            last_ids.push_back(i);
        }

        return ret;
    }
    return ret;
}

Eigen::Vector3d HeadPoseDetector::estimate_ground_speed_by_tracker(double z, cv::Rect2d roi, cv::Point3f track_spd) {
    double roi_x = roi.x + roi.width/2;
    double roi_y = roi.y + roi.height/2;

    double roi_x_new = roi_x + track_spd.x * dt;
    double roi_y_new = roi_y + track_spd.y * dt;

    std::vector<cv::Point2f> pts{cv::Point2f(roi_x_new, roi_y_new)};
    std::vector<cv::Point2f> pts_un;
    cv::undistortPoints(pts, pts_un, settings->K, settings->D);
    Eigen::Vector3d point_new_3d(pts_un[0].x*z, pts_un[0].y*z, z);
    
    std::vector<cv::Point2f> pts_roi{cv::Point2f(roi_x, roi_y)};
    std::vector<cv::Point2f> pts_roi_un;
    cv::undistortPoints(pts_roi, pts_roi_un, settings->K, settings->D);

    Eigen::Vector3d point_3d(pts_roi_un[0].x*z, pts_roi_un[0].y*z, z);

    // We can use this velocity to recgonize the deadzone. Thus, lock the attitude
    Eigen::Vector3d gspd = (point_new_3d - point_3d)/dt;

    return gspd;
}


void HeadPoseDetector::draw(cv::Mat & frame, cv::Rect2d roi, cv::Rect2d face_roi, cv::Rect2d fsa_roi, CvPts landmarks, Pose p, cv::Point3f track_spd) {
    //Head or face ROI
    //cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2);

    //FSANet ROI
    if (settings->use_fsa) {
        cv::rectangle(frame, fsa_roi, cv::Scalar(255, 255, 255), 2);
        //cv::rectangle(frame, face_roi, cv::Scalar(0, 255, 0), 2);
    }

    for (auto pt: landmarks) {
        cv::circle(frame, pt, 1, cv::Scalar(0, 255, 0), -1);
    }

    cv::Mat tvec, Rmat, rvec;
    Eigen::Matrix3d _R = p.R()*Rface.transpose();
    cv::eigen2cv(p.pos(), tvec);
    cv::eigen2cv(_R, Rmat);
    cv::Rodrigues(Rmat, rvec);
    cv::drawFrameAxes(frame, settings->K, cv::Mat(), rvec, -tvec, 0.05, 3);

    //    cv::Point2f center(face_roi.x + face_roi.width/2, face_roi.y + face_roi.height/2);
    //    cv::arrowedLine(frame,  center, center+cv::Point2f(track_spd.x, track_spd.y), cv::Scalar(0, 127, 255), 3);

    //    char info[100] = {0};
    //    auto eul = quat2eulers(dq, true);
    //    sprintf(info, "dYPR [%3.1f,%3.1f,%3.1f]", eul(0), eul(1), eul(2));
    //    cv::putText(frame, info, cv::Point2f(20, 150), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

}

std::pair<bool, Pose> HeadPoseDetector::solve_face_pose(CvPts landmarks, std::vector<cv::Point3f> landmarks_3d, cv::Mat & frame, Eigen::Vector3d fsa_ypr) {
    Eigen::Vector3d T;
    Eigen::Matrix3d R;
    std::vector<int> indices{ 0,1,15,16,27,28,29,30,31,32,33,34,35,36,39,42,45 };

//    if (fsa_ypr(0) > DEG2RAD*10) {
//        indices.erase(indices.begin() + 1);
//        indices.erase(indices.begin());
//    }

//    if (fsa_ypr(0) < -DEG2RAD*10) {
//        indices.erase(indices.begin() + 3);
//        indices.erase(indices.begin() + 2);
//    }

    std::vector<uchar> pts_mask(landmarks_3d.size());

    for (auto i : indices) {
        pts_mask[i] = 1;
    }

    reduceVector(landmarks, pts_mask);
    reduceVector(landmarks_3d, pts_mask);

    if (landmarks.size() == 0) {
        return make_pair(false, Pose(T, R));
    }

    auto rvec = rvec_init.clone();
    auto tvec = tvec_init.clone();

    TicToc tic;
    std::vector<uchar> inlier(landmarks.size());
    bool success = cv::solvePnP(landmarks_3d, landmarks, settings->K, settings->D, rvec, tvec, true);

    if (tic.toc() > 30) {
        qDebug() << "PnP Time" << tic.toc();
    }


    cv::cv2eigen(tvec, T);
    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);

    cv::cv2eigen(Rcv, R);
    T = -T;

    cv::drawFrameAxes(frame, settings->K, cv::Mat(), rvec, tvec, 0.1, 1);

    CvPts reproject_landmarks;
    cv::projectPoints(landmarks_3d, rvec, tvec, settings->K, settings->D, reproject_landmarks);

    for (unsigned int i = 0; i < landmarks.size(); i++) {
        auto pt = landmarks[i];
//        char info[100] = {0};
//        sprintf(info, "%d", indices[i]);
//        cv::putText(frame, info, pt - cv::Point2f(0, 5), cv::FONT_HERSHEY_COMPLEX, 0.3, cv::Scalar(255, 0, 0), 1);
        cv::circle(frame, pt, 3, cv::Scalar(0, 0, 255), 1);
    }

    if (success) {
        if (first_solve_pose) {
            first_solve_pose = false;
        }

        //char info[100] = {0};
        //sprintf(info, "Tpnp [%3.1f,%3.1f,%3.1f] cm", T.x()*100, T.y()*100, T.z()*100);
        //cv::putText(frame, info, cv::Point2f(20, 100), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    } else {
        qDebug() << "pnp Solve failed";
    }

    return make_pair(success, Pose(T, R));
}
