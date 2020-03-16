#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <FlightAgxSettings.h>
#include <QAction>
#include <QMenu>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    hd.start();
    this->start_camera_preview();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_show() {
    start_camera_preview();
   QTimer::singleShot(0, this, SLOT(show()));
}

void MainWindow::on_exit() {
   QCoreApplication::quit();
}

void MainWindow::create_tray_icon() {
    if (m_tray_icon == nullptr) {
        m_tray_icon = new QSystemTrayIcon(QIcon(":/icon.png"), this);

        connect( m_tray_icon, SIGNAL(activated(QSystemTrayIcon::ActivationReason)), this, SLOT(on_show_hide(QSystemTrayIcon::ActivationReason)) );

        QAction *quit_action = new QAction( "Exit", m_tray_icon );
        connect( quit_action, SIGNAL(triggered()), this, SLOT(on_exit()) );

        QAction *hide_action = new QAction( "Show", m_tray_icon );
        connect( hide_action, SIGNAL(triggered()), this, SLOT(on_show()) );

        connect(m_tray_icon, SIGNAL(DoubleClick()), this, SLOT(on_show()));
        connect(m_tray_icon,SIGNAL(activated(QSystemTrayIcon::ActivationReason)),
                this ,SLOT(iconActivated(QSystemTrayIcon::ActivationReason)));

        QMenu *tray_icon_menu = new QMenu;
        tray_icon_menu->addAction( hide_action );
        tray_icon_menu->addAction( quit_action );

        m_tray_icon->setContextMenu( tray_icon_menu );

        m_tray_icon->show();
    }
}

void MainWindow::iconActivated(QSystemTrayIcon::ActivationReason reason) {
    switch (reason) {
       case QSystemTrayIcon::Trigger:
       case QSystemTrayIcon::DoubleClick:
            this->on_show();
       default:
        break;
    }
}

void MainWindow::DisplayImage() {
    cv::Mat & img = hd.get_preview_image();
    QImage imdisplay((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_BGR888); //Converts the CV image into Qt standard format
    ui->preview_camera->setPixmap(QPixmap::fromImage(imdisplay));//display the image in label that is created earlier
}

void MainWindow::on_startButton_clicked()
{
    hd.start();
    this->start_camera_preview();
}

void MainWindow::on_endbutton_clicked()
{
    hd.stop();
    this->stop_camera_preview();
}

void MainWindow::start_camera_preview() {
    Timer = new QTimer(this);
    settings->enable_preview = true;
    connect(Timer, SIGNAL(timeout()), this, SLOT(DisplayImage()));
    Timer->start(30);
    camera_preview_enabled = true;
}

void MainWindow::stop_camera_preview() {
    settings->enable_preview = false;
    camera_preview_enabled = false;
    Timer->stop();
}

void MainWindow::on_toggle_preview_clicked()
{
    if (camera_preview_enabled) {
        this->stop_camera_preview();
    } else {
        this->start_camera_preview();
    }
}

void MainWindow::on_gamemode_clicked()
{
    this->stop_camera_preview();
    create_tray_icon();
    QTimer::singleShot(10, this, SLOT(hide()));
}
