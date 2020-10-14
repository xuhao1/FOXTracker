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

private:
    Ui::AgentXConfig *ui;
};

#endif // AGENTXCONFIG_H
