#include "mainwindow.h"
#include "HeadPoseDetector.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include<QDebug>
#include "FlightAgxSettings.h"

FlightAgxSettings * settings;

int main(int argc, char *argv[])
{
    qInfo() << "Here we are";
    settings = new FlightAgxSettings;

    QApplication a(argc, argv);
    MainWindow w;
    HeadPoseDetector hd;
    hd.start();
    w.show();
    return a.exec();
}
