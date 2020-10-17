#include "FlightAgxSettings.h"
#include <iostream>
#include <fstream>

void FlightAgxSettings::load_from_config_yaml() {
    qDebug() << "Read config at " << cfg_name.c_str() << "\n";
    config = YAML::LoadFile(cfg_name);
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

    landmark_detect_method = config["landmark_detect_method"].as<int>();

    ekf_predict_dt = config["ekf_predict_dt"].as<double>();
    use_ekf = config["use_ekf"].as<bool>();
    disp_duration = config["disp_duration"].as<double>();
    disp_max_series_size = config["disp_max_series_size"].as<int>();
    fsa_pnp_mixture_rate = config["fsa_pnp_mixture_rate"].as<double>();

    hotkey_joystick_names.resize(2);
    hotkey_joystick_buttons.resize(2);

    hotkey_joystick_names[0] = config["hotkey_joystick_name0"].as<std::string>();
    hotkey_joystick_buttons[0] = config["hotkey_joystick_button0"].as<int>();

    hotkey_joystick_names[1] = config["hotkey_joystick_name1"].as<std::string>();
    hotkey_joystick_buttons[1] = config["hotkey_joystick_button1"].as<int>();

    pitch_offset_fsa_pnp = config["pitch_offset_fsa_pnp"].as<float>();

}


void FlightAgxSettings::write_to_file() {
    config["hotkey_joystick_name0"] = hotkey_joystick_names[0];
    config["hotkey_joystick_button0"] = hotkey_joystick_buttons[0];

    config["hotkey_joystick_name1"] = hotkey_joystick_names[1];
    config["hotkey_joystick_button1"] = hotkey_joystick_buttons[1];

    std::ofstream fout(cfg_name.c_str());
    fout << config;
    fout.close();
    qDebug() << "Succesful write config to file";
}
