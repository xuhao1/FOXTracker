#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <HeadPoseDetector.h>
#include <QTimer>
#include <QSystemTrayIcon>
#include <PoseDataSender.h>
#include "agentxconfig.h"
#include "poseremapper.h"
#include <QShortcut>
#include "uglobalhotkeys.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

    HeadPoseDetector hd;
    PoseDataSender data_sender;
    PoseRemapper remapper;
    bool is_running = false;

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void on_show();
    void on_exit();

private slots:
    void on_startButton_clicked();

    void on_endbutton_clicked();

    void on_toggle_preview_clicked();

    void on_gamemode_clicked();
    void iconActivated(QSystemTrayIcon::ActivationReason reason);

    void DisplayImage();

    void on_pose6d_data(double t, Pose6DoF _pose);

    void on_config_button_clicked();

    void on_center_keyboard_event();

    void handle_global_hotkeys(unsigned int _id);

    void on_pause_clicked();

    void on_centerButton_clicked();

private:
    QTimer* Timer;
    bool camera_preview_enabled = false;
    void start_camera_preview();
    void stop_camera_preview();
    Ui::MainWindow *ui;
    void create_tray_icon();
    QSystemTrayIcon * m_tray_icon = nullptr;

    AgentXConfig * config_menu = nullptr;

    QShortcut * m_globalShortcut;
    UGlobalHotkeys *hotkeyManager;
};
#endif // MAINWINDOW_H
