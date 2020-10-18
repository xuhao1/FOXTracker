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
}

void HeadPoseTrackDetectWorker::run() {
    is_running = true;
//    hd->run_detect_thrad();
}

void HeadPoseTrackDetectWorker::stop() {
    is_running = false;

}

void HeadPoseDetector::loop() {
    TicToc tic_cap;
    Mat frame;
    cap >> frame;

//    qDebug() << "Capture takes" << tic_cap.toc();

    if( frame.empty() ) {
        qDebug() << "Empty frame"    ;
        return;
    }
    double t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
    dt = t - last_t;
    last_t = t;
    TicToc tic;
    auto ret = detect_head_pose(frame, t, dt);
    auto poses_raw = ret.second;
    Pose pose;
    Pose pose_raw;

    if (ret.first) {
        if (poses_raw.size() == 1) {
            pose_raw = poses_raw[0];
        } else {
            pose_raw = (poses_raw[0].slerp(settings->fsa_pnp_mixture_rate, poses_raw[1]));
        }

        TicToc tic;
        if(settings->use_ekf) {
            // qDebug("P0 %f %f %f", poses_raw[0].pos().x(), poses_raw[0].pos().y(), poses_raw[0].pos().z());
            qDebug("Q0 %f %f %f %f", poses_raw[0].att().x(), poses_raw[0].att().y(), poses_raw[0].att().z(), poses_raw[0].att().w());
            ekf.on_raw_pose_data(t, Rcam*poses_raw[0], 0);
            if (poses_raw.size() > 1) {
                // ekf.on_raw_pose_data(t, poses_raw[1], 1);
                // qDebug("P1 %f %f %f", poses_raw[1].pos().x(), poses_raw[1].pos().y(), poses_raw[1].pos().z());
            }
        } else {
            pose = pose_raw;
        }
        this->on_detect_pose6d_raw(t, make_pair(R2ypr(pose_raw.R()), pose_raw.pos()));

        inited = true;
    }

    t = QDateTime::currentMSecsSinceEpoch()/1000.0 - t0;
    TicToc tic_ekf;

    if(inited && settings->use_ekf) {
        pose = Rcam.transpose() * ekf.predict(t);
        qDebug("QEKF %f %f %f %f", pose.att().x(), pose.att().y(), pose.att().z(), pose.att().w());
    }
    
    // this->on_detect_P(t, ekf.getP());

    auto R = pose.R();
    auto T = pose.pos();

    //This pose is in world frame
    if (ret.first || (settings->use_ekf && inited)) {
        this->on_detect_pose(t, make_pair(pose.R(), pose.pos()));
        this->on_detect_pose6d(t, make_pair(R2ypr(R), T));
        this->on_detect_twist(t, ekf.get_angular_velocity(), ekf.get_linear_velocity());
    }

    if (settings->enable_preview) {
        frame.copyTo(preview_image);
    }

//    qDebug() << "Loop takes" << tic_cap.toc();

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

    qDebug() << "Start Timer with fps of" << settings->fps << "Cap FPS"<< cap.get(cv::CAP_PROP_FPS);
    main_loop_timer = new QTimer;
    main_loop_timer->moveToThread(&mainThread);
    connect(main_loop_timer, SIGNAL(timeout()), this, SLOT(loop()), Qt::DirectConnection);

    main_loop_timer->start(1/settings->fps*1000);
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
        cap.release();
    }
}

void HeadPoseDetector::reset() {
    first_solve_pose = true;
    stop();
    start();
}



std::pair<bool, std::vector<Pose>> HeadPoseDetector::detect_head_pose(cv::Mat & frame, double t, double dt) {
    TicToc tic;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
    CvPts landmarks;
    std::vector<cv::Point3f> landmarks_3d;
    cv::Mat frame_clean;
    cv::Rect2d roi;
    cv::Rect2d fsa_roi;
    cv::Rect2d face_roi;
    cv::Point2f track_spd(0, 0);

    Eigen::Vector3d fsa_ypr;

    std::vector<Pose> detected_poses;

    frame_count ++;
    frame_clean = frame.clone();
    if (first_solve_pose) {
        roi = fd->detect(frame, last_roi);
        if (roi.area() > MIN_ROI_AREA) {
            tracker = create_tracker();
            tracker->init(frame, roi);
        } else {
            return make_pair(false, std::vector<Pose>());
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
//            if (cv::norm(track_spd) > 10) {
//               qDebug() << "Track SPD [" << track_spd.x << "," << track_spd.y << "]" << " ROI [" << roi.x
//                        << "," << roi.y << "]" << " last ROI [" << last_roi.x << "," << last_roi.y <<"]"
//                        << " dt " << dt;
//            }
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
            return make_pair(false, std::vector<Pose>());
        }

    }

    if (roi.area() < MIN_ROI_AREA) {
        return make_pair(false, std::vector<Pose>());
    }

    face_roi = roi;
    //Use FSA To Detect Rotation
    TicToc fsa;
    if(settings->use_fsa) {
        fsa_roi = crop_roi(roi, frame, 0.2);
        static Rect2d fsa_roi_last;
        if (fsa_roi_last.area() < MIN_ROI_AREA) {
            fsa_roi_last = fsa_roi;
        } else {
            fsa_roi = fsa_roi_last = mixture_roi(fsa_roi_last, fsa_roi, settings->roi_filter_rate);
        }

        if (fsa_roi.area() > MIN_ROI_AREA) {
            auto fsa_ypr_raw = fsanet.inference(frame(fsa_roi));
//            qDebug("EUL FIX Y %f P %f",eul_by_crop(roi)(0), eul_by_crop(roi)(1), eul_by_crop(roi)(2));
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
    auto lmd_ret = lmd->detect(frame, crop_roi(face_roi, frame, 0));
    landmarks = lmd_ret.first;
    landmarks_3d = lmd_ret.second;

    if (landmarks.size() != landmarks_3d.size()) {
        qDebug("Landmark detection failed. 2D pts %d 3D pts %d", landmarks.size(), landmarks_3d.size());
        return make_pair(false, std::vector<Pose>());
    }

    if (frame_count % ((int)settings->fps) == 0)
        qDebug() << "Landmark detector cost " << tic1.toc() << "FSA " << dt_fsa;

    TicToc ticpnp;
    auto ret = this->solve_face_pose(landmarks, landmarks_3d, frame);

    //Estimate Planar speed of face with tracker

//    double z = ret.second.pos().z() + settings->cervical_face_mm/1000;
    double z = ret.second.pos().z();
    double roi_x = roi.x + roi.width/2;
    double roi_y = roi.y + roi.height/2;

    double roi_x_new = roi_x + track_spd.x * dt;
    double roi_y_new = roi_y + track_spd.y * dt;

//    std::vector<cv::Point2f> pts{cv::Point2f(roi_x_new, roi_y_new)};
//    std::vector<cv::Point3f> pts3d;
//    cv::undistortPoints(pts, pts3d, settings->K, settings->D);
//    cv::Point3f pt3d = pts3d[0] * z;
//    Eigen::Vector3d point_new_3d(pt3d.x, pt3d.y, pt3d.z);
//    Eigen::Vector3d point_3d = ret.second.pos();
//    point_3d.z() = z;
    //We can use this velocity to recgonize the deadzone. Thus, lock the attitude
//    Eigen::Vector3d vel_planar_face = (point_new_3d - point_3d)/dt;

//    qDebug("Vel planar face %f %f %f", vel_planar_face.x(), vel_planar_face.y(), vel_planar_face.z());

    if (ret.first) {
        auto pose = ret.second;
        T = pose.pos();
        pose.att() = pose.att()*Rface;
        detected_poses.push_back(pose);

        //Use FSA YPR Here
        if (settings->use_fsa) {
             R = Rcam.inverse() * Eigen::AngleAxisd(fsa_ypr(0), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(fsa_ypr(1) + settings->pitch_offset_fsa_pnp, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(-fsa_ypr(2), Eigen::Vector3d::UnitX());
            Eigen::Quaterniond qR(R);
            detected_poses.push_back(Pose(T, qR));

            //The mismatch here is a bug of the new model
            dq = pose.att()*qR.inverse();
        }

        if (settings->enable_preview) {
            draw(frame, roi, face_roi, fsa_roi, landmarks, Pose(T, R), track_spd);
        }

        last_clean_frame = frame_clean;
        last_landmark_pts = landmarks;
        last_ids.clear();
        for (int i = 0; i < 68; i++) {
            last_ids.push_back(i);
        }


        return make_pair(true, detected_poses);
    }
    return make_pair(false, detected_poses);
}

void HeadPoseDetector::draw(cv::Mat & frame, cv::Rect2d roi, cv::Rect2d face_roi, cv::Rect2d fsa_roi, CvPts landmarks, Pose p, cv::Point2f track_spd) {
    //Head or face ROI
    cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2);

    //FSANet ROI
    if (settings->use_fsa) {
        cv::rectangle(frame, fsa_roi, cv::Scalar(255, 255, 255), 2);
        cv::rectangle(frame, face_roi, cv::Scalar(0, 255, 0), 2);
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

    cv::Point2f center(face_roi.x + face_roi.width/2, face_roi.y + face_roi.height/2);
    cv::arrowedLine(frame,  center, center+track_spd, cv::Scalar(0, 127, 255), 3);

    char info[100] = {0};
    auto eul = quat2eulers(dq, true);
    sprintf(info, "dYPR [%3.1f,%3.1f,%3.1f]", eul(0), eul(1), eul(2));
    cv::putText(frame, info, cv::Point2f(20, 150), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);

}

std::pair<bool, Pose> HeadPoseDetector::solve_face_pose(CvPts landmarks, std::vector<cv::Point3f> landmarks_3d, cv::Mat & frame) {
    Eigen::Vector3d T;
    Eigen::Matrix3d R;

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

    cv::drawFrameAxes(frame, settings->K, cv::Mat(), rvec, tvec, 0.1, 1);

    cv::cv2eigen(tvec, T);
    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);

    cv::cv2eigen(Rcv, R);
    T = -T;

    for (size_t i =0; i < landmarks.size(); i++) {
        if(inlier[i]) {
            cv::circle(frame, landmarks[i], 5, cv::Scalar(255, 255, 0), 1);
        }
    }

    if (success) {
        if (first_solve_pose) {
            first_solve_pose = false;
        }

        char info[100] = {0};
        sprintf(info, "Tpnp [%3.1f,%3.1f,%3.1f] cm", T.x()*100, T.y()*100, T.z()*100);
        cv::putText(frame, info, cv::Point2f(20, 100), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    } else {
        qDebug() << "pnp Solve failed";
    }

    return make_pair(success, Pose(T, R));
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
