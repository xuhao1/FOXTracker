#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <HeadPoseDetector.h>
#include <QTimer>
#include <QSystemTrayIcon>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    HeadPoseDetector hd;
public slots:
    void DisplayImage();
    void on_show();
    void on_exit();
private slots:
    void on_startButton_clicked();

    void on_endbutton_clicked();

    void on_toggle_preview_clicked();

    void on_gamemode_clicked();
    void iconActivated(QSystemTrayIcon::ActivationReason reason);
private:
    QTimer* Timer;
    bool camera_preview_enabled = false;
    void start_camera_preview();
    void stop_camera_preview();
    Ui::MainWindow *ui;
    void create_tray_icon();
    QSystemTrayIcon * m_tray_icon = nullptr;
};
#endif // MAINWINDOW_H
