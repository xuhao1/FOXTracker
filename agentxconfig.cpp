#include "agentxconfig.h"
#include "ui_agentxconfig.h"

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
