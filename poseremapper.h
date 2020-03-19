#ifndef POSEREMAPPER_H
#define POSEREMAPPER_H

#include <QObject>
#include "fagx_datatype.h"

class PoseRemapper : public QObject
{
    Q_OBJECT

    Pose initial_pose;
    bool is_inited = false;

    Eigen::Matrix3d Rcam;
public:
    explicit PoseRemapper(QObject *parent = nullptr);

signals:
    void send_mapped_posedata(double t, Pose6DoF _pose);
public slots:
    void on_pose_data(double t, Pose pose);
    void reset_center();
};

#endif // POSEREMAPPER_H
