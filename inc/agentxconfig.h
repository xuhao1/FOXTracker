#ifndef AGENTXCONFIG_H
#define AGENTXCONFIG_H

#include <QDialog>
#include <ekfconfig.h>
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

signals:
    void reset_camera();

private:
    Ui::AgentXConfig *ui;
};

#endif // AGENTXCONFIG_H
