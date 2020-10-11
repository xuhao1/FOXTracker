#include "agentxconfig.h"
#include "ui_agentxconfig.h"
#include "FlightAgxSettings.h"

AgentXConfig::AgentXConfig(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::AgentXConfig)
{
    ui->setupUi(this);
}

AgentXConfig::~AgentXConfig()
{
    delete ui;
}

EKFConfig * AgentXConfig::ekf_config_menu() {
    return  ui->ekfconfig;
}


void FlightAgxSettings::load_from_config_yaml() {
    qDebug() << "Read config at " << cfg_name.c_str() << "\n";
    YAML::Node config = YAML::LoadFile(cfg_name);
    detect_duration = config["detect_duration"].as<double>();
    camera_id = config["camera_id"].as<int>();
    enable_multithread_detect = config["enable_multithread_detect"].as<bool>();
    retrack_queue_size = config["retrack_queue_size"].as<int>();
    fps = config["fps"].as<double>();
    send_posedata_udp = config["send_posedata_udp"].as<bool>();
    port = config["port"].as<int>();
    udp_host = config["udp_host"].as<std::string>();
    qDebug() << "Will send to" << udp_host.c_str();
    use_ft = config["use_ft"].as<bool>();
    use_npclient = config["use_npclient"].as<bool>();
    cov_Q = config["cov_Q"].as<double>();
    cov_T = config["cov_T"].as<double>();

    cov_V = config["cov_V"].as<double>();
    cov_W = config["cov_W"].as<double>();

    detect_method = config["detect_method"].as<int>();

    ekf_predict_dt = config["ekf_predict_dt"].as<double>();
    use_ekf = config["use_ekf"].as<bool>();
    disp_duration = config["disp_duration"].as<double>();
    disp_max_series_size = config["disp_max_series_size"].as<int>();
}
