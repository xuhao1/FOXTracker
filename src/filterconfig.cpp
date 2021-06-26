#include <filterconfig.h>
#include "ui_filterconfig.h"
#include "FlightAgxSettings.h"

FilterConfig::FilterConfig(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FilterConfig)
{
    ui->setupUi(this);

    if(settings->use_accela) {
        ui->Accela_Check->setCheckState(Qt::Checked);
    } else {
        ui->Accela_Check->setCheckState(Qt::Unchecked);
    }

    if (settings->double_accela) {
        ui->Accela_Check_2->setCheckState(Qt::Checked);
    } else {
        ui->Accela_Check_2->setCheckState(Qt::Unchecked);
    }

    ui->rot_smooth_sld->setSliderPosition(range_v_inv(settings->accela_s.rot_smoothing, rot_smooth_min, rot_smooth_max)*100);

    ui->rot_deadzone_sld->setSliderPosition(range_v_inv(settings->accela_s.rot_deadzone, rot_deadzone_min, rot_deadzone_max)*100);

    ui->trans_smooth_sld->setSliderPosition(range_v_inv(settings->accela_s.pos_smoothing, trans_smooth_min, trans_smooth_max)*100);
    
    ui->trans_deadzone_sld->setSliderPosition(range_v_inv(settings->accela_s.pos_deadzone, trans_deadzone_min, trans_deadzone_max)*100);

    ui->rot_expo_sld_yaw->setSliderPosition(range_v_inv(settings->expo_eul(0), expo_min, expo_max)*100);
    ui->rot_expo_sld_pitch->setSliderPosition(range_v_inv(settings->expo_eul(1), expo_min, expo_max)*100);
    ui->rot_expo_sld_roll->setSliderPosition(range_v_inv(settings->expo_eul(2), expo_min, expo_max)*100);

    ui->trans_expo_sld_z->setSliderPosition(range_v_inv(settings->expo_trans.z(), expo_min, expo_max)*100);
    ui->trans_expo_sld_y->setSliderPosition(range_v_inv(settings->expo_trans.y(), expo_min, expo_max)*100);
    ui->trans_expo_sld_x->setSliderPosition(range_v_inv(settings->expo_trans.x(), expo_min, expo_max)*100);


    ui->rot_input_max_sld_yaw->setSliderPosition(range_v_inv(settings->inp_bound_eul(0), rot_input_min, rot_input_max)*100);
    ui->rot_input_max_sld_pitch->setSliderPosition(range_v_inv(settings->inp_bound_eul(1), rot_input_min, rot_input_max)*100);
    ui->rot_input_max_sld_roll->setSliderPosition(range_v_inv(settings->inp_bound_eul(2), rot_input_min, rot_input_max)*100);

    ui->trans_input_max_sld_z->setSliderPosition(range_v_inv(settings->inp_bound_trans(0), trans_input_min, trans_input_max)*100);
    ui->trans_input_max_sld_y->setSliderPosition(range_v_inv(settings->inp_bound_trans(1), trans_input_min, trans_input_max)*100);
    ui->trans_input_max_sld_x->setSliderPosition(range_v_inv(settings->inp_bound_trans(2), trans_input_min, trans_input_max)*100);

    ui->rot_output_max_sld_yaw->setSliderPosition(range_v_inv(settings->out_bound_eul(0), rot_input_min, rot_input_max)*100);
    ui->rot_output_max_sld_pitch->setSliderPosition(range_v_inv(settings->out_bound_eul(1), rot_input_min, rot_input_max)*100);
    ui->rot_output_max_sld_roll->setSliderPosition(range_v_inv(settings->out_bound_eul(2), rot_input_min, rot_input_max)*100);

    ui->trans_output_max_sld_z->setSliderPosition(range_v_inv(settings->out_bound_trans(0), trans_output_min, trans_output_max)*100);
    ui->trans_output_max_sld_y->setSliderPosition(range_v_inv(settings->out_bound_trans(1), trans_output_min, trans_output_max)*100);
    ui->trans_output_max_sld_x->setSliderPosition(range_v_inv(settings->out_bound_trans(2), trans_output_min, trans_output_max)*100);
}


FilterConfig::~FilterConfig()
{
    delete ui;
}

void FilterConfig::on_rot_smooth_sld_valueChanged(int value)
{  
    double v = value/100.0;
    settings->accela_s.rot_smoothing = range_v(v, rot_smooth_min, rot_smooth_max);
    settings->set_value<double>("accela_rot_smoothing", settings->accela_s.rot_smoothing);
    ui->rot_smooth_label->setText(QString::number(settings->accela_s.rot_smoothing));
}

void FilterConfig::on_rot_deadzone_sld_valueChanged(int value)
{
    double v = value/100.0;
    settings->accela_s.rot_deadzone = range_v(v, rot_deadzone_min, rot_deadzone_max);
    settings->set_value<double>("accela_rot_deadzone", settings->accela_s.rot_deadzone);
    ui->rot_deadzone_label->setText(QString::number(settings->accela_s.rot_deadzone));
}

void FilterConfig::on_trans_smooth_sld_valueChanged(int value)
{
    double v = value/100.0;
    settings->accela_s.pos_smoothing = range_v(v, trans_smooth_min, trans_smooth_max);
    settings->set_value<double>("accela_pos_smoothing", settings->accela_s.pos_smoothing);
    ui->trans_smooth_label->setText(QString::number(settings->accela_s.pos_smoothing));
}

void FilterConfig::on_trans_deadzone_sld_valueChanged(int value)
{
    double v = value/100.0;
    settings->accela_s.pos_deadzone = range_v(v, trans_deadzone_min, trans_deadzone_max);
    settings->set_value<double>("accela_pos_deadzone", settings->accela_s.pos_deadzone);
    ui->trans_deadzone_label->setText(QString::number(settings->accela_s.pos_deadzone));
}

void FilterConfig::on_rot_input_max_sld_yaw_valueChanged(int value)
{
    double v = value/100.0;
    settings->inp_bound_eul(0) = range_v(v, rot_input_min, rot_input_max);
    settings->set_value<double>("inp_bound_yaw", settings->inp_bound_eul(0));
    ui->rot_input_max_label_yaw->setText(QString::number(settings->inp_bound_eul(0)));
}

void FilterConfig::on_rot_output_max_sld_yaw_valueChanged(int value)
{
    double v = value/100.0;
    settings->out_bound_eul(0)= range_v(v, rot_output_min, rot_output_max);
    settings->set_value<double>("out_bound_yaw", settings->out_bound_eul(0));
    ui->rot_output_max_label_yaw->setText(QString::number(settings->out_bound_eul(0)));
}

void FilterConfig::on_rot_expo_sld_yaw_valueChanged(int value)
{
    double v = value/100.0;
    settings->expo_eul(0) = range_v(v, expo_min, expo_max);
    settings->set_value<double>("expo_eul_yaw", settings->expo_eul(0));
    ui->rot_expo_label_yaw->setText(QString::number(settings->expo_eul(0)));
}

void FilterConfig::on_rot_input_max_sld_pitch_valueChanged(int value)
{
    double v = value/100.0;
    settings->inp_bound_eul(1) = range_v(v, rot_input_min, rot_input_max);
    settings->set_value<double>("inp_bound_pitch", settings->inp_bound_eul(1));
    ui->rot_input_max_label_pitch->setText(QString::number(settings->inp_bound_eul(1)));
}

void FilterConfig::on_rot_output_max_sld_pitch_valueChanged(int value)
{
    double v = value/100.0;
    settings->out_bound_eul(1)= range_v(v, rot_output_min, rot_output_max);
    settings->set_value<double>("out_bound_pitch", settings->out_bound_eul(1));
    ui->rot_output_max_label_pitch->setText(QString::number(settings->out_bound_eul(1)));
}

void FilterConfig::on_rot_expo_sld_pitch_valueChanged(int value)
{
    double v = value/100.0;
    settings->expo_eul(1) = range_v(v, expo_min, expo_max);
    settings->set_value<double>("expo_eul_pitch", settings->expo_eul(1));
    ui->rot_expo_label_pitch->setText(QString::number(settings->expo_eul(1)));
}

void FilterConfig::on_rot_input_max_sld_roll_valueChanged(int value)
{
    double v = value/100.0;
    settings->inp_bound_eul(2) = range_v(v, rot_input_min, rot_input_max);
    settings->set_value<double>("inp_bound_roll", settings->inp_bound_eul(2));
    ui->rot_input_max_label_roll->setText(QString::number(settings->inp_bound_eul(2)));
}

void FilterConfig::on_rot_output_max_sld_roll_valueChanged(int value)
{
    double v = value/100.0;
    settings->out_bound_eul(2)= range_v(v, rot_output_min, rot_output_max);
    settings->set_value<double>("out_bound_roll", settings->out_bound_eul(2));
    ui->rot_output_max_label_roll->setText(QString::number(settings->out_bound_eul(2)));
}

void FilterConfig::on_rot_expo_sld_roll_valueChanged(int value)
{
    double v = value/100.0;
    settings->expo_eul(2) = range_v(v, expo_min, expo_max);
    settings->set_value<double>("expo_eul_roll", settings->expo_eul(2));
    ui->rot_expo_label_roll->setText(QString::number(settings->expo_eul(2)));
}

void FilterConfig::on_trans_input_max_sld_z_valueChanged(int value)
{
    double v = value/100.0;
    settings->inp_bound_trans.z() = range_v(v, trans_input_min, trans_input_max);
    settings->set_value<double>("inp_bound_z", settings->inp_bound_trans.z());
    ui->trans_input_max_label_z->setText(QString::number(settings->inp_bound_trans.z()*100));
}

void FilterConfig::on_trans_output_max_sld_z_valueChanged(int value)
{
    double v = value/100.0;
    settings->out_bound_trans.z() = range_v(v, trans_output_min, trans_output_max);
    settings->set_value<double>("out_bound_z", settings->out_bound_trans.z());
    ui->trans_output_max_label_z->setText(QString::number(settings->out_bound_trans.z()*100));
}

void FilterConfig::on_trans_expo_sld_z_valueChanged(int value)
{
    double v = value/100.0;
    settings->expo_trans.x() = range_v(v, expo_min, expo_max);
    settings->set_value<double>("expo_trans_x", settings->expo_trans.x());
    ui->trans_expo_label_z->setText(QString::number(settings->expo_trans.x()));
}

void FilterConfig::on_trans_input_max_sld_y_valueChanged(int value)
{
    double v = value/100.0;
    settings->inp_bound_trans.y() = range_v(v, trans_input_min, trans_input_max);
    settings->set_value<double>("inp_bound_y", settings->inp_bound_trans.y());
    ui->trans_input_max_label_y->setText(QString::number(settings->inp_bound_trans.y()*100));
}

void FilterConfig::on_trans_output_max_sld_y_valueChanged(int value)
{
    double v = value/100.0;
    settings->out_bound_trans.y() = range_v(v, trans_output_min, trans_output_max);
    settings->set_value<double>("out_bound_y", settings->out_bound_trans.y());
    ui->trans_output_max_label_y->setText(QString::number(settings->out_bound_trans.y()*100));
}

void FilterConfig::on_trans_expo_sld_y_valueChanged(int value)
{
    double v = value/100.0;
    settings->expo_trans.y() = range_v(v, expo_min, expo_max);
    settings->set_value<double>("expo_trans_y", settings->expo_trans.y());
    ui->trans_expo_label_y->setText(QString::number(settings->expo_trans.y()));
}

void FilterConfig::on_trans_input_max_sld_x_valueChanged(int value)
{
    double v = value/100.0;
    settings->inp_bound_trans.x() = range_v(v, trans_input_min, trans_input_max);
    settings->set_value<double>("inp_bound_x", settings->inp_bound_trans.x());
    ui->trans_input_max_label_x->setText(QString::number(settings->inp_bound_trans.x()*100));
}

void FilterConfig::on_trans_output_max_sld_x_valueChanged(int value)
{
    double v = value/100.0;
    settings->out_bound_trans.x() = range_v(v, trans_output_min, trans_output_max);
    settings->set_value<double>("out_bound_x", settings->out_bound_trans.x());
    ui->trans_output_max_label_x->setText(QString::number(settings->out_bound_trans.x()*100));
}

void FilterConfig::on_trans_expo_sld_x_valueChanged(int value)
{
    double v = value/100.0;
    settings->expo_trans.x() = range_v(v, expo_min, expo_max);
    settings->set_value<double>("expo_trans_x", settings->expo_trans.x());
    ui->trans_expo_label_x->setText(QString::number(settings->expo_trans.x()));
}

void FilterConfig::on_Accela_Check_stateChanged(int arg1)
{
    settings->use_accela = arg1;
    settings->set_value<bool>("use_accela", settings->use_accela);
}

void FilterConfig::on_Accela_Check_2_stateChanged(int arg1)
{
    settings->double_accela = arg1;
    settings->set_value<bool>("double_accela", settings->double_accela);
}
