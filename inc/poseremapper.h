#ifndef POSEREMAPPER_H
#define POSEREMAPPER_H

#include <QObject>
#include "fagx_datatype.h"
#include <filter_accela.h>
#include <QDateTime>

class PoseRemapper : public QObject
{
    Q_OBJECT

    Pose initial_pose;
    bool is_inited = false;
    accela _accela, _accela2;

    Eigen::Matrix3d Rcam;
    QTimer * pose_callback_timer;
    double t_last;
    Eigen::Vector3d eul_last, T_last;
    double t0;

public:
    explicit PoseRemapper(QObject *parent = nullptr);

signals:
    void send_mapped_posedata(double t, Pose6DoF _pose);
public slots:
    void on_pose_data(double t, Pose_ pose);
    void reset_center();

private slots:
    void pose_callback_loop();
};

#endif // POSEREMAPPER_H
