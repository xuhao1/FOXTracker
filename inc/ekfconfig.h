#ifndef EKFCONFIG_H
#define EKFCONFIG_H

#include <QWidget>
#include <Eigen/Eigen>
#include <fagx_datatype.h>
#include <QSplineSeries>
#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>
#include <QTimer>
#include <QScatterSeries>

QT_CHARTS_USE_NAMESPACE

namespace Ui {
class EKFConfig;
}

class EKFConfig : public QWidget
{
    Q_OBJECT

    double qcov_min = 0.001;
    double qcov_max = 1.0;

    double tcov_min = 0.001;
    double tcov_max = 0.5;

    double wcov_min = 0.001;
    double wcov_max = 10.0;

    double vcov_min = 0.001;
    double vcov_max = 10.0;

    QtCharts::QLineSeries * angle_splines[3] = {0};
    QtCharts::QLineSeries * T_splines[3] = {0};
    QtCharts::QLineSeries * Pt_splines[3] = {0};
    QtCharts::QLineSeries * angle_raw_splines[3] = {0};
    QtCharts::QLineSeries * Traw_splines[3] = {0};
    QtCharts::QLineSeries * w_splines[3] = {0};
    QtCharts::QLineSeries * v_splines[3] = {0};
    double last_update_t = 0;

    QChart * ekf_disp_charts[6];

    int activate_chart = 0;

    QValueAxis *axisX;

    Eigen::Vector3d Tinit;
    bool inited = false;


    void initalize_angle_charts();
    void initalize_translation_charts();
public:
    explicit EKFConfig(QWidget *parent = nullptr);
    ~EKFConfig();

    void setQNoise(double cov_q);
    void setTNoise(double cov_q);
    void setVNoise(double cov_V);
    void setWNoise(double cov_W);

    void reset();
    QTimer* Timer;
public slots:
    void on_detect_twist(double t, Eigen::Vector3d w, Eigen::Vector3d v);
    void on_detect_pose6d(double t, Pose6DoF pose);
    void on_detect_pose6d_raw(double t, Pose6DoF pose);
    void on_Pmat(double t, Matrix13d P);
    void set_activate_chart(int _chart);

private slots:
    void on_qnoise_slider_valueChanged(int value);

    void on_tnoise_slider_valueChanged(int value);

    void on_wnoise_slider_valueChanged(int value);

    void on_vnoise_slider_valueChanged(int value);

    void update_plot();

    void on_comboBox_currentIndexChanged(int index);

private:
    Ui::EKFConfig *ui;
};

#endif // EKFCONFIG_H
