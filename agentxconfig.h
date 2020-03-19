#ifndef AGENTXCONFIG_H
#define AGENTXCONFIG_H

#include <QDialog>

namespace Ui {
class AgentXConfig;
}

class AgentXConfig : public QDialog
{
    Q_OBJECT

public:
    explicit AgentXConfig(QWidget *parent = nullptr);
    ~AgentXConfig();

private:
    Ui::AgentXConfig *ui;
};

#endif // AGENTXCONFIG_H
