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
            pose_raw = Rcam*poses_raw[0];
        } else {
            pose_raw = Rcam*(poses_raw[0].slerp(settings->fsa_pnp_mixture_rate, poses_raw[1]));
        }

        TicToc tic;
        if(settings->use_ekf) {
            pose = ekf.on_raw_pose_data(t, Rcam*poses_raw[0], 0);
            if (poses_raw.size() > 1) {
                pose = ekf.on_raw_pose_data(t, Rcam*poses_raw[1], 1);
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
        pose = ekf.predict(t);
    }
    this->on_detect_P(t, ekf.getP());

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
           if(fc % 10 == 0)
               qDebug() << "Frontal face detector cost" << tic.toc() << "ms";
           //Track to now image

           if (roi.width < 1 && roi.height < 1) {
               frame_pending_detect = false;
               qDebug() << "Detect failed in thread";
               Sleep(10);
               continue;
           }

           cv::Ptr<cv::Tracker> tracker = TrackerMOSSE::create();
           detect_mtx.lock();

           tracker->init(_frame, roi);
           bool success_track = true;

//           qDebug() << "Track " << frames.size() << "frames";
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
    if(!cap.open(settings->camera_id)) {
        qDebug() << "Not able to open camera" << settings->camera_id <<  "exiting";
        preview_image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
        char warn[100] = {0};
        sprintf(warn, "Camera ID %d Error. Change in config.yaml!!!", settings->camera_id);
        cv::putText(preview_image, warn, cv::Point2f(20, 240), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
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

    Eigen::Vector3d fsa_ypr;

    std::vector<Pose> detected_poses;

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
                return make_pair(false, std::vector<Pose>());
            }

        }

        if (roi.width < 1 || roi.height < 1) {
            return make_pair(false, std::vector<Pose>());
        }

        face_roi = roi;
        //Use FSA To Detect Rotation
        TicToc fsa;
        if(settings->use_fsa) {
            fsa_roi = crop_roi(roi, frame);
            if (fsa_roi.area() > 0) {
                auto fsa_ypr_raw = fsanet.inference(frame(fsa_roi));
                if (fsa_ypr_raw(0) > 0) {
                    face_roi.x = face_roi.x - fsa_ypr_raw(0)*face_roi.width*0.3;
                }
                face_roi.width = face_roi.width + fabs(fsa_ypr_raw(0))*face_roi.width*0.3;
                fsa_ypr = fsa_ypr_raw - eul_by_crop(fsa_roi);
            }
        }
        double dt_fsa = fsa.toc();

        TicToc tic1;
        landmarks = lmd->detect(frame, face_roi);
        if (frame_count % ((int)settings->fps) == 0)
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
        T = pose.pos();
        pose.att() = pose.att()*Rface;

        detected_poses.push_back(pose);
        //Use FSA YPR Here
        if (settings->use_fsa) {
             R = Rcam.inverse() * Eigen::AngleAxisd(fsa_ypr(0), Eigen::Vector3d::UnitZ())
                * Eigen::AngleAxisd(fsa_ypr(1), Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(-fsa_ypr(2), Eigen::Vector3d::UnitX());
            Eigen::Quaterniond qR(R);
            detected_poses.push_back(Pose(T, qR));
        }

        if (settings->enable_preview) {
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
            Eigen::Matrix3d _R = R*Rface.inverse();
            cv::eigen2cv(T, tvec);
            cv::eigen2cv(_R, Rmat);
            cv::Rodrigues(Rmat, rvec);
            cv::drawFrameAxes(frame, settings->K, cv::Mat(), rvec, tvec, 0.05, 3);

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

std::pair<bool, Pose> HeadPoseDetector::solve_face_pose(CvPts landmarks, std::vector<cv::Point3f> landmarks_3d, cv::Mat & frame) {
    Eigen::Vector3d T;
    Eigen::Matrix3d R;

    if (landmarks.size() == 0) {
        return make_pair(false, Pose(T, R));
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

    cv::Mat Rcv;
    cv::Rodrigues(rvec, Rcv);

    cv::cv2eigen(tvec/1000.0, T);
    cv::cv2eigen(Rcv, R);
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
