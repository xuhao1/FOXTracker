#ifndef FILTERCONFIG_H
#define FILTERCONFIG_H

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
class FilterConfig;
}

class FilterConfig : public QWidget
{
    Q_OBJECT

    double rot_smooth_min = 0.01;
    double rot_smooth_max = 0.2;

    double rot_deadzone_min = 0.01;
    double rot_deadzone_max = 3.0;

    double trans_smooth_min = 0.01;
    double trans_smooth_max = 0.2;

    double trans_deadzone_min = 0.0;
    double trans_deadzone_max = 0.2;

    double rot_input_min = 10;
    double rot_input_max = 45;

    double rot_output_min = 30;
    double rot_output_max = 180;

    double trans_input_min = 0.10;
    double trans_input_max = 1.0;

    double trans_output_min = 0.30;
    double trans_output_max = 1.0;

    double expo_min = 0.0;
    double expo_max = 1.0;

public:
    explicit FilterConfig(QWidget *parent = nullptr);
    ~FilterConfig();

private slots:
    void on_rot_smooth_sld_valueChanged(int value);

    void on_rot_deadzone_sld_valueChanged(int value);

    void on_trans_smooth_sld_valueChanged(int value);

    void on_trans_deadzone_sld_valueChanged(int value);

    void on_rot_input_max_sld_yaw_valueChanged(int value);

    void on_rot_output_max_sld_yaw_valueChanged(int value);

    void on_rot_expo_sld_yaw_valueChanged(int value);

    void on_rot_input_max_sld_pitch_valueChanged(int value);

    void on_rot_output_max_sld_pitch_valueChanged(int value);

    void on_rot_expo_sld_pitch_valueChanged(int value);

    void on_rot_input_max_sld_roll_valueChanged(int value);

    void on_rot_output_max_sld_roll_valueChanged(int value);

    void on_rot_expo_sld_roll_valueChanged(int value);

    void on_trans_input_max_sld_z_valueChanged(int value);

    void on_trans_output_max_sld_z_valueChanged(int value);

    void on_trans_expo_sld_z_valueChanged(int value);

    void on_trans_input_max_sld_y_valueChanged(int value);

    void on_trans_output_max_sld_y_valueChanged(int value);

    void on_trans_expo_sld_y_valueChanged(int value);

    void on_trans_input_max_sld_x_valueChanged(int value);

    void on_trans_output_max_sld_x_valueChanged(int value);

    void on_trans_expo_sld_x_valueChanged(int value);

private:
    Ui::FilterConfig *ui;
};

#endif // FILTERCONFIG_H
