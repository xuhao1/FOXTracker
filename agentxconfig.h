#ifndef AGENTXCONFIG_H
#define AGENTXCONFIG_H

#include <QDialog>
#include <ekfconfig.h>

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
private:
    Ui::AgentXConfig *ui;
};

#endif // AGENTXCONFIG_H
