#include "HeadPoseDetector.h"
using namespace cv;

void HeadPoseDetector::run_thread() {
    VideoCapture cap;
    if(!cap.open(0))
        return;
    while(is_running)
    {
        Mat frame;
        cap >> frame;
        if( frame.empty() ) break;
        imshow("this is you, smile! :)", frame);
        if( waitKey(10) == 27 ) break;
    }
}
