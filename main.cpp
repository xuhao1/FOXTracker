#include "mainwindow.h"
#include "HeadPoseDetector.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include<QDebug>
#include "FlightAgxSettings.h"

FlightAgxSettings * settings;

int main(int argc, char *argv[])
{

    qRegisterMetaType<Pose>("Pose");
    qRegisterMetaType<Pose6DoF>("Pose6DoF");

    qInfo() << "Here we are";
    settings = new FlightAgxSettings;
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
