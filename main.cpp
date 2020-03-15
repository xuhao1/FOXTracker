#include "mainwindow.h"
#include "HeadPoseDetector.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include<QDebug>

int main(int argc, char *argv[])
{
    qInfo() << "Here we are";

    QApplication a(argc, argv);
    MainWindow w;
    HeadPoseDetector hd;
    hd.start();
    w.show();
    return a.exec();
}
