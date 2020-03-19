#include "mainwindow.h"
#include "HeadPoseDetector.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include<QDebug>
#include "FlightAgxSettings.h"

FlightAgxSettings * settings = nullptr;

int main(int argc, char *argv[])
{

    qRegisterMetaType<Pose>("Pose");
    qRegisterMetaType<Pose6DoF>("Pose6DoF");

    qInfo() << "Welcome! plilots";
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication a(argc, argv);

    settings = new FlightAgxSettings;

    MainWindow w;
    w.show();
    return a.exec();
}
