#include "agentxconfig.h"
#include "ui_agentxconfig.h"
#include "FlightAgxSettings.h"
#include <string>
#include <QMessageBox>

AgentXConfig::AgentXConfig(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AgentXConfig)
{
    ui->setupUi(this);
    qDebug() << "Start AgentX Config page";
    QStringList joystickNames = QJoysticks::getInstance()->deviceNames();
    qDebug() << "Joysticks" << joystickNames;
    QList<QJoystickDevice*> joysticks = QJoysticks::getInstance()->inputDevices();
    qDebug() << "Joysicks" << joysticks.size();

    QJoysticks::getInstance()->setVirtualJoystickEnabled (true);
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

    ui->SlerpRate_Input->setValue(settings->fsa_pnp_mixture_rate*100);

    ui->Hotkey1_Joystick->setText(settings->hotkey_joystick_names[0].c_str());
    ui->Hotkey1_Button->setText(QString::number(settings->hotkey_joystick_buttons[0]));

    ui->Hotkey2_Joystick->setText(settings->hotkey_joystick_names[1].c_str());
    ui->Hotkey2_Button->setText(QString::number(settings->hotkey_joystick_buttons[1]));

    ui->FSAPnPOffset_disp->setDecMode();
    ui->FSAPnPOffset_disp->setDigitCount(5);
    ui->FSAPnPOffset_disp->display(settings->pitch_offset_fsa_pnp*RAD2DEG);
    ui->FSAPnPOffset_input->setValue(settings->pitch_offset_fsa_pnp*RAD2DEG/20*100);

    ui->LandmarkModel_input->setValue(settings->landmark_detect_method);
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

    qDebug() << "Save config.yaml to file";
    settings->write_to_file();
}

void AgentXConfig::on_SlerpRate_Input_valueChanged(int value)
{
    settings->fsa_pnp_mixture_rate = ((double)value)/100;
    settings->set_value<double>("fsa_pnp_mixture_rate", settings->fsa_pnp_mixture_rate);

    qDebug() << "fsa_pnp_mixture_rate" << settings->fsa_pnp_mixture_rate;
}

void AgentXConfig::on_Bind_HotKey_clicked(int key) {
    mbox = new QMessageBox();
    mbox->setText("Press any key to bind re-center key");
    mbox->setStandardButtons(QMessageBox::Cancel);
    wait_for_bind = key;
    int ret = mbox->exec();
    qDebug() << "MBox recturn" << ret;
}

void AgentXConfig::buttonEvent (const QJoystickButtonEvent& event) {
    std::string joyname = event.joystick->name.toUtf8().constData();
    int btn_id =  event.button;
    if(event.pressed && wait_for_bind >= 0) {
        qDebug() << "Joystick" << event.joystick->name << " pressed " << event.button << " pressed" << event.pressed;
        qDebug() << "Bind button to hotkey" << wait_for_bind;
        settings->hotkey_joystick_names[wait_for_bind] = joyname;
        settings->hotkey_joystick_buttons[wait_for_bind] = event.button;

        if (mbox!=nullptr) {
            mbox->done(0);
        }
        wait_for_bind = -1;
        return;
    }

    if (event.pressed) {
        for (size_t i = 0; i < settings->hotkey_joystick_names.size(); i++) {
            if(settings->hotkey_joystick_names[i] == joyname &&
               settings->hotkey_joystick_buttons[i] == btn_id) {
                qDebug() << "Hotkey" << i << "pressed";
                recenter_hotkey_pressed();
            }
            //Only trigger first hotkey function.
            return;
        }
    }
}


void AgentXConfig::on_Bind_HotKey2_clicked() {
    on_Bind_HotKey_clicked(1);
    ui->Hotkey2_Joystick->setText(settings->hotkey_joystick_names[1].c_str());
    ui->Hotkey2_Button->setText(QString::number(settings->hotkey_joystick_buttons[1]));
}

void AgentXConfig::on_Bind_HotKey1_clicked() {
    on_Bind_HotKey_clicked(0);
    ui->Hotkey1_Joystick->setText(settings->hotkey_joystick_names[0].c_str());
    ui->Hotkey1_Button->setText(QString::number(settings->hotkey_joystick_buttons[0]));
}

void AgentXConfig::on_FSAPnPOffset_input_valueChanged(int value)
{
    float v = ((float)value)/100;
    float offset_degree = v * 20;
    float offset = offset_degree*DEG2RAD;
    settings->pitch_offset_fsa_pnp = offset;
    settings->set_value("pitch_offset_fsa_pnp", offset);
    ui->FSAPnPOffset_disp->display(offset_degree);
}

void AgentXConfig::on_LandmarkModel_input_valueChanged(int value)
{
    settings->landmark_detect_method = value;
    settings->set_value("landmark_detect_method", value);
}
