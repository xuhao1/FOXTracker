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

    ui->qnoise_slider->setSliderPosition(log_v_inv(settings->cov_Q, qcov_min, qcov_max)*100);
    ui->tnoise_slider->setSliderPosition(log_v_inv(settings->cov_T, tcov_min, tcov_max)*100);
    ui->vnoise_slider->setSliderPosition(log_v_inv(settings->cov_V, vcov_min, vcov_max)*100);
    ui->wnoise_slider->setSliderPosition(log_v_inv(settings->cov_W, wcov_min, wcov_max)*100);

    this->initalize_angle_charts();
    this->initalize_translation_charts();

    Timer = new QTimer(this);
    connect(Timer, SIGNAL(timeout()), this, SLOT(update_plot()));
}

void EKFConfig::update_plot() {
    if(this->isVisible()) {
//        ekf_disp_charts[activate_chart]->update();
    }
}

void EKFConfig::initalize_translation_charts() {
    std::string T_names[3] = {
        "X",
        "Y",
        "X"
    };

    for (int i = 0; i < 3; i ++) {
        T_splines[i] = new QLineSeries();
        T_splines[i]->setName(T_names[i].c_str());


        Traw_splines[i] = new QLineSeries();
        Traw_splines[i]->setName((T_names[i] + "Raw").c_str());


        v_splines[i] = new QLineSeries();
        v_splines[i]->setName((T_names[i] + "Velocity").c_str());

        auto chart = ekf_disp_charts[i + 3] = new QChart;

        chart->setTitle((T_names[i] + "Angle and Angular Velocity").c_str());

        axisX = new QValueAxis;
        axisX->setTickCount(10);
        chart->addAxis(axisX, Qt::AlignBottom);

        chart->addSeries(T_splines[i]);
        chart->addSeries(Traw_splines[i]);
        chart->addSeries(v_splines[i]);
        chart->createDefaultAxes();
        chart->axes(Qt::Vertical).first()->setRange(-50, 50);
        chart->axes(Qt::Horizontal).first()->setRange(0, 30);
        chart->legend()->setAlignment(Qt::AlignBottom);
    }

    this->set_activate_chart(0);
}

void EKFConfig::set_activate_chart(int _chart) {
    this->activate_chart = _chart;
    this->ui->chart->setChart(ekf_disp_charts[_chart]);
}

void EKFConfig::initalize_angle_charts() {
    std::string angle_names[3] = {
        "Yaw",
        "Pitch",
        "Roll"
    };

    for (int i = 0; i < 3; i ++) {
        angle_splines[i] = new QLineSeries();
        angle_splines[i]->setName(angle_names[i].c_str());
        angle_raw_splines[i] = new QLineSeries();
        angle_raw_splines[i]->setName((angle_names[i] + "Raw").c_str());
        w_splines[i] = new QLineSeries();
        w_splines[i]->setName((angle_names[i] + "Rate").c_str());

        auto chart = ekf_disp_charts[i] = new QChart;

        chart->setTitle((angle_names[i] + "Angle and Angular Velocity").c_str());

        axisX = new QValueAxis;
        axisX->setTickCount(10);
        chart->addAxis(axisX, Qt::AlignBottom);

        chart->addSeries(angle_splines[i]);
        chart->addSeries(w_splines[i]);
        chart->addSeries(angle_raw_splines[i]);
        chart->createDefaultAxes();
        chart->axes(Qt::Vertical).first()->setRange(-30, 30);
        chart->axes(Qt::Horizontal).first()->setRange(0, 30);
        chart->legend()->setAlignment(Qt::AlignBottom);
    }


    this->ui->chart->setChart(ekf_disp_charts[0]);
}

void EKFConfig::on_detect_twist(double t, Eigen::Vector3d w, Eigen::Vector3d v) {
    if (this->isVisible()) {
        v = settings->Rcam.transpose()*v;
        w = settings->Rcam.transpose()*w;
        w_splines[0]->append(t, w.z()*180/3.1415);
        w_splines[1]->append(t, w.y()*180/3.1415);
        w_splines[2]->append(t, w.x()*180/3.1415);

        v_splines[0]->append(t, v.x()*100);
        v_splines[1]->append(t, v.y()*100);
        v_splines[2]->append(t, v.z()*100);

        if (v_splines[0]->count() > settings->disp_max_series_size * 1.5) {
            while (v_splines[0]->count() > settings->disp_max_series_size ) {
                for (int i = 0; i < 3; i++) {
                    v_splines[i]->remove(0);
                    w_splines[i]->remove(0);
                }
            }
        }
    }
}

void EKFConfig::on_detect_pose6d(double t, Pose6DoF pose) {
    if (this->isVisible()) {
        if (!inited) {
            Tinit = pose.second;
            inited = true;
        }
        auto T = settings->Rcam.transpose()*(pose.second - Tinit);
        auto eul = pose.first;
        double yaw = eul.x();

        angle_splines[0]->append(t, yaw);
        angle_splines[1]->append(t, eul.y());
        angle_splines[2]->append(t, eul.z());

        T_splines[0]->append(t, T.x()*100);
        T_splines[1]->append(t, T.y()*100);
        T_splines[2]->append(t, T.z()*100);

        if (angle_splines[0]->count() > settings->disp_max_series_size) {
            for (int i = 0; i < 3; i++) {
                angle_splines[i]->remove(0);
                T_splines[i]->remove(0);
            }
        }

        if (t > settings->disp_duration && t - last_update_t > settings->disp_duration/3) {
            for (int i = 0; i < 6; i ++) {
                ekf_disp_charts[i]->axes(Qt::Horizontal).first()->setRange(t - settings->disp_duration*2.0/3.0, t + t - settings->disp_duration/3);
            }
            last_update_t = t;
        }
    }

}

void EKFConfig::on_Pmat(double t, Matrix13d P) {
//    Pt_splines[0]->append(t, P(11, 11)*10000);
}

void EKFConfig::on_detect_pose6d_raw(double t, Pose6DoF pose) {
    if (this->isVisible()) {
        //qDebug() << "Adding raw data";
        if (!inited) {
            Tinit = pose.second;
            inited = true;
        }
        auto T = settings->Rcam.transpose()*(pose.second - Tinit);
        auto eul = pose.first;
        double yaw = eul.x();

        angle_raw_splines[0]->append(t, yaw);
        angle_raw_splines[1]->append(t, eul.y());
        angle_raw_splines[2]->append(t, eul.z());

        Traw_splines[0]->append(t, T.x()*100);
        Traw_splines[1]->append(t, T.y()*100);
        Traw_splines[2]->append(t, T.z()*100);


        if (angle_raw_splines[0]->count() > settings->disp_max_series_size) {
            for (int i = 0; i < 3; i++) {
                Traw_splines[i]->remove(0);
                angle_raw_splines[i]->remove(0);
            }
        }

        if (t > settings->disp_duration && t - last_update_t > settings->disp_duration/3) {
            for (int i = 0; i < 6; i ++) {
                ekf_disp_charts[i]->axes(Qt::Horizontal).first()->setRange(t - settings->disp_duration*2.0/3.0, t + t - settings->disp_duration/3);
            }
            last_update_t = t;
        }
    }

}
void EKFConfig::setQNoise(double cov_q) {
    qDebug() << "Set Q Noise" << cov_q;
    settings->cov_Q = cov_q;
    settings->set_value<double>("cov_Q", cov_q);
    ui->QNoise->setText(QString::number(cov_q));
}

void EKFConfig::setTNoise(double cov_t) {
    settings->cov_T = cov_t;
    settings->set_value<double>("cov_T", cov_t);
    ui->TNoise->setText(QString::number(cov_t));
}

void EKFConfig::setVNoise(double cov_V) {
    settings->cov_V = cov_V;
    settings->set_value<double>("cov_V", cov_V);
    ui->VNoise->setText(QString::number(cov_V));
}

void EKFConfig::setWNoise(double cov_W){
    settings->cov_W = cov_W;
    settings->set_value<double>("cov_W", cov_W);
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

void EKFConfig::on_comboBox_currentIndexChanged(int index)
{
    this->set_activate_chart(index);
}
