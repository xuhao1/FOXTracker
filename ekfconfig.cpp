#include "ekfconfig.h"
#include "ui_ekfconfig.h"
#include "FlightAgxSettings.h"
#include <QtCharts/QChartView>

#include <QSplineSeries>

QT_CHARTS_USE_NAMESPACE

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

    chart = new QChart();
    chart->setTitle("Yaw Angle and Angular Velocity");

    axisX = new QValueAxis;
    axisX->setTickCount(10);
    chart->addAxis(axisX, Qt::AlignBottom);



    T_splines[0] = new QSplineSeries();
    T_splines[0]->setName("X");
    v_splines[0] = new QSplineSeries();
    v_splines[0]->setName("Vx");

    angle_splines[0] = new QSplineSeries();
    angle_splines[0]->setName("Yaw");
    angle_raw_splines[0] = new QSplineSeries();
    angle_raw_splines[0]->setName("YawRaw");
    w_splines[0] = new QSplineSeries();
    w_splines[0]->setName("YawRate");

    angle_splines[1] = new QSplineSeries();
    angle_splines[1]->setName("Pitch");
    angle_raw_splines[1] = new QSplineSeries();
    angle_raw_splines[1]->setName("PitchRaw");
    w_splines[1] = new QSplineSeries();
    w_splines[1]->setName("PitchRate");

    /*
    chart->addSeries(angle_splines[0]);
    chart->addSeries(w_splines[0]);
    chart->addSeries(angle_raw_splines[0]);*/
    /*
    chart->addSeries(angle_splines[1]);
    chart->addSeries(w_splines[1]);
    chart->addSeries(angle_raw_splines[1]);*/

    chart->addSeries(T_splines[0]);
    chart->addSeries(v_splines[0]);

    chart->setTitle("Yaw");
    chart->createDefaultAxes();
    chart->axes(Qt::Vertical).first()->setRange(-30, 30);
    chart->axes(Qt::Horizontal).first()->setRange(0, 30);
    chart->legend()->setAlignment(Qt::AlignBottom);
    chart->setAnimationOptions(QChart::AllAnimations);
    this->ui->chart->setChart(chart);
}

void EKFConfig::on_detect_twist(double t, Eigen::Vector3d w, Eigen::Vector3d v) {
    v = settings->Rcam.transpose()*v;
    w_splines[0]->append(t, w.z()*180/3.1415);
    v_splines[0]->append(t, v.x()*100);
}

void EKFConfig::on_detect_pose6d(double t, Pose6DoF pose) {
    if (!inited) {
        Tinit = pose.second;
        inited = true;
    }
    auto T = settings->Rcam.transpose()*(pose.second - Tinit);
    auto eul = pose.first;
    double yaw = eul.x();
    angle_splines[0]->append(t, yaw);
    T_splines[0]->append(t, T.x()*100);
    angle_splines[1]->append(t, eul.y());
    //T_splines[1]->append(t, pose.second.y()*100);
    if (t > 30 && t - last_update_t > 10) {
        chart->axes(Qt::Horizontal).first()->setRange(t - 20, t + 10);
        last_update_t = t;
    }

}


void EKFConfig::on_detect_pose6d_raw(double t, Pose6DoF pose) {
    auto eul = pose.first;
    double yaw = eul.x();
    angle_raw_splines[0]->append(t, yaw);
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



void EKFConfig::reset() {

}
