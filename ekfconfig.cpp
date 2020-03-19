#include "ekfconfig.h"
#include "ui_ekfconfig.h"
#include "FlightAgxSettings.h"

double log_v(double v, double min, double max) {
    return min*exp(v*log(max/ min));
}

double log_v_inv(double value, double min, double max) {
    return log(value / min)/log(max/min);
}

EKFConfig::EKFConfig(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EKFConfig)
{
    ui->setupUi(this);
    setQNoise(settings->cov_Q);
    setTNoise(settings->cov_T);
    setVNoise(settings->cov_V);
    setWNoise(settings->cov_W);

    ui->qnoise_slider->setSliderPosition(log_v_inv(settings->cov_Q, qcov_min, qcov_max)*100);
    ui->tnoise_slider->setSliderPosition(log_v_inv(settings->cov_T, tcov_min, tcov_max)*100);
    ui->vnoise_slider->setSliderPosition(log_v_inv(settings->cov_V, vcov_min, vcov_max)*100);
    ui->wnoise_slider->setSliderPosition(log_v_inv(settings->cov_W, wcov_min, wcov_max)*100);
}



void EKFConfig::setQNoise(double cov_q) {
    qDebug() << "Set Q Noise" << cov_q;
    settings->cov_Q = cov_q;
    ui->QNoise->setText(QString::number(cov_q));
}

void EKFConfig::setTNoise(double cov_t) {
    settings->cov_T = cov_t;
    ui->TNoise->setText(QString::number(cov_t));
}

void EKFConfig::setVNoise(double cov_V) {
    settings->cov_V = cov_V;
    ui->VNoise->setText(QString::number(cov_V));
}

void EKFConfig::setWNoise(double cov_W){
    settings->cov_W = cov_W;
    ui->WNoise->setText(QString::number(cov_W));
}

EKFConfig::~EKFConfig()
{
    delete ui;
}


void EKFConfig::on_qnoise_slider_valueChanged(int value)
{
    double v = value / 100.0;
    setQNoise(log_v(v, qcov_min, qcov_max));
}

void EKFConfig::on_tnoise_slider_valueChanged(int value)
{
    double v = value / 100.0;
    setTNoise(log_v(v, tcov_min, tcov_max));
}

void EKFConfig::on_wnoise_slider_valueChanged(int value)
{
    double w = value / 100.0;
    setWNoise(log_v(w, wcov_min, wcov_max));
}

void EKFConfig::on_vnoise_slider_valueChanged(int value)
{
    double v = value / 100.0;
    setVNoise(log_v(v, vcov_min, vcov_max));
}
