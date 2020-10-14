#include "agentxconfig.h"
#include "ui_agentxconfig.h"
#include "FlightAgxSettings.h"
#include <string>

AgentXConfig::AgentXConfig(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AgentXConfig)
{
    //QJoysticks::getInstance()->setVirtualJoystickEnabled (false);
    ui->setupUi(this);
    qDebug() << "Start AgentX Config page";
    QStringList joystickNames = QJoysticks::getInstance()->deviceNames();
    qDebug() << "Joysticks" << joystickNames;
    QList<QJoystickDevice*> joysticks = QJoysticks::getInstance()->inputDevices();
    qDebug() << "Joysicks" << joysticks.size();

    connect(QJoysticks::getInstance(), &QJoysticks::buttonEvent, this, &AgentXConfig::buttonEvent);

    if(settings->use_ekf) {
        ui->EKF_Check->setCheckState(Qt::Checked);
    } else {
        ui->EKF_Check->setCheckState(Qt::Unchecked);
    }

    if(settings->use_ft || settings->use_npclient) {
        ui->DCtrl_Check->setCheckState(Qt::Checked);
    } else {
        ui->DCtrl_Check->setCheckState(Qt::Unchecked);
    }

    if(settings->send_posedata_udp) {
        ui->SendUDP_Check->setCheckState(Qt::Checked);
    } else {
        ui->SendUDP_Check->setCheckState(Qt::Unchecked);
    }

    ui->Port_Input->setValue(settings->port);
    ui->FPS_Input->setValue(settings->fps);
    ui->CameraID_Input->setValue(settings->camera_id);
    ui->DetectDura_Input->setValue(settings->detect_duration);
    ui->IP_Input->setText(settings->udp_host.c_str());
}

void AgentXConfig::buttonEvent (const QJoystickButtonEvent& event) {
    qDebug() << "Joystick" << event.joystick->name << " pressed " << event.button << " pressed" << event.pressed;
}

AgentXConfig::~AgentXConfig()
{
    delete ui;
}

EKFConfig * AgentXConfig::ekf_config_menu() {
    return  ui->ekfconfig;
}


void AgentXConfig::on_EKF_Check_stateChanged(int arg1)
{
    settings->use_ekf = arg1;
    settings->set_value<bool>("use_ekf", arg1);
}

void AgentXConfig::on_DCtrl_Check_stateChanged(int arg1)
{
    settings->use_ft = settings->use_npclient = arg1;
    settings->set_value<bool>("use_ft", arg1);
    settings->set_value<bool>("use_npclient", arg1);
}

void AgentXConfig::on_SendUDP_Check_stateChanged(int arg1)
{
    settings->send_posedata_udp = arg1;
    settings->set_value<bool>("send_posedata_udp", arg1);
}

void AgentXConfig::on_buttonBox_accepted()
{

    if(settings->udp_host != ui->IP_Input->text().toUtf8().constData() || settings->port!=ui->Port_Input->value()) {
        settings->udp_host = ui->IP_Input->text().toUtf8().constData();
        settings->set_value<std::string>("udp_host", settings->udp_host);
        settings->port = ui->Port_Input->value();
        settings->set_value<int>("port", settings->port);

        //TODO:Reset IP here
    }

    if (ui->CameraID_Input->value() != settings->camera_id || settings->fps != ui->FPS_Input->value()) {
        settings->camera_id = ui->CameraID_Input->value();
        settings->set_value<int>("camera_id", settings->camera_id);

        settings->fps = ui->FPS_Input->value();
        settings->set_value<double>("fps", settings->fps);
        reset_camera();
    }

    settings->detect_duration = ui->DetectDura_Input->value();
    settings->set_value<int>("detect_duration", settings->detect_duration);

    settings->write_to_file();
}
