#ifndef AGENTXCONFIG_H
#define AGENTXCONFIG_H

#include <QDialog>
#include <ekfconfig.h>
#include <QMessageBox>
//#include <
#include <QJoysticks.h>

namespace Ui {
class AgentXConfig;
}

class AgentXConfig : public QDialog
{
    Q_OBJECT

public:
    explicit AgentXConfig(QWidget *parent = nullptr);
    ~AgentXConfig();
    EKFConfig * ekf_config_menu();

    void buttonEvent (const QJoystickButtonEvent& event);

private slots:
    void on_EKF_Check_stateChanged(int arg1);

    void on_DCtrl_Check_stateChanged(int arg1);

    void on_SendUDP_Check_stateChanged(int arg1);

    void on_buttonBox_accepted();

    void on_SlerpRate_Input_valueChanged(int value);

    void on_Bind_HotKey2_clicked();

    void on_Bind_HotKey1_clicked();

    void on_Bind_HotKey_clicked(int key);

    void on_FSAPnPOffset_input_valueChanged(int value);

signals:
    void reset_camera();
    void recenter_hotkey_pressed();

private:
    Ui::AgentXConfig *ui;
    QMessageBox * mbox = nullptr;
    int wait_for_bind = -1;
};

#endif // AGENTXCONFIG_H
