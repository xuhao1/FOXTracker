#ifndef EKFCONFIG_H
#define EKFCONFIG_H

#include <QWidget>

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
    double wcov_max = 1.0;

    double vcov_min = 0.001;
    double vcov_max = 2.0;

public:
    explicit EKFConfig(QWidget *parent = nullptr);
    ~EKFConfig();

    void setQNoise(double cov_q);
    void setTNoise(double cov_q);
    void setVNoise(double cov_V);
    void setWNoise(double cov_W);

private slots:
    void on_qnoise_slider_valueChanged(int value);

    void on_tnoise_slider_valueChanged(int value);

    void on_wnoise_slider_valueChanged(int value);

    void on_vnoise_slider_valueChanged(int value);

private:
    Ui::EKFConfig *ui;
};

#endif // EKFCONFIG_H
