QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG += c++11
# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += C:\Users\plane\source\opencv\opencv-build-86\install\include
INCLUDEPATH += C:\Users\plane\source\eigen-3.3.7
INCLUDEPATH += C:\Users\plane\source\dlib\

CONFIG += force_debug_info
RC_ICONS = icon.ico

#win32:CONFIG(debug, debug|release): LIBS += C:\Users\plane\source\opencv\build\x64\vc14\lib\opencv_world420d.lib
win32:CONFIG(debug, debug|release): LIBS += C:\Users\plane\source\dlib\build\dlib\Debug\dlib19.19.99_debug_32bit_msvc1916.lib
win32:CONFIG(release, debug|release): LIBS += C:\Users\plane\source\dlib\build\dlib\RelWithDebInfo\dlib19.19.99_relwithdebinfo_32bit_msvc1916.lib

#win32:CONFIG(release, debug|release): LIBS += -LC:\Users\plane\source\opencv\opencv-build-86\install\x86\vc15\lib \
#    -lopencv_core420 -lopencv_highgui420 -lopencv_tracking420 -lopencv_video420 -lopencv_imgproc420

win32:CONFIG(release, debug|release): LIBS += -LC:\Users\plane\source\opencv\opencv-build-86\install\x86\vc15\lib \
    -lopencv_core420 -lopencv_highgui420 -lopencv_tracking420 -lopencv_video420 -lopencv_imgproc420 -lopencv_videoio420 -lopencv_calib3d420

win32:CONFIG(debug, debug|release): LIBS += -LC:\Users\plane\source\opencv\opencv-build-86\install\x86\vc15\lib \
    -lopencv_core420d -lopencv_highgui420d -lopencv_tracking420d -lopencv_video420d -lopencv_imgproc420d -lopencv_videoio420d -lopencv_calib3d420d


SOURCES += \
    HeadPoseDetector.cpp \
    PoseDataSender.cpp \
    main.cpp \
    mainwindow.cpp

HEADERS += \
    FlightAgxSettings.h \
    HeadPoseDetector.h \
    PoseDataSender.h \
    mainwindow.h

FORMS += \
    mainwindow.ui

TRANSLATIONS += \
    FlightAgentX_en_US.ts

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    main.qrc
