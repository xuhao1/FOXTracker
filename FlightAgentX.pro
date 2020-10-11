QT       += core gui network charts
QT += widgets

CONFIG += c++17
# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += C:\Users\plane\source\opencv-4.4.0\build\install\include
INCLUDEPATH += C:\Users\plane\source\eigen-3.3.7
INCLUDEPATH += C:\Users\plane\source\dlib\
INCLUDEPATH += C:\Users\plane\Develop\UGlobalHotkey
INCLUDEPATH += "C:\Program Files (x86)\YAML_CPP\include"
INCLUDEPATH += ./inc/
INCLUDEPATH += C:\Users\plane\source\onnxruntime-win-x86-1.5.1\include
CONFIG += force_debug_info
RC_ICONS = icon.ico

#win32:CONFIG(debug, debug|release): LIBS += C:\Users\plane\source\opencv\build\x64\vc14\lib\opencv_world420d.lib
win32:CONFIG(debug, debug|release): LIBS += C:\Users\plane\source\dlib\build\dlib\Debug\dlib19.19.99_debug_32bit_msvc1916.lib
win32:CONFIG(release, debug|release): LIBS += C:\Users\plane\source\dlib\build\dlib\RelWithDebInfo\dlib19.19.99_relwithdebinfo_32bit_msvc1916.lib

#win32:CONFIG(release, debug|release): LIBS += -LC:\Users\plane\source\opencv\opencv-build-86\install\x86\vc15\lib \
#    -lopencv_core420 -lopencv_highgui420 -lopencv_tracking420 -lopencv_video420 -lopencv_imgproc420

win32:CONFIG(release, debug|release): LIBS += -LC:\Users\plane\source\opencv-4.4.0\build\install\x86\vc15\lib \
    -lopencv_core440 -lopencv_highgui440 -lopencv_tracking440 -lopencv_video440 -lopencv_imgproc440 -lopencv_videoio440 -lopencv_calib3d440 -lopencv_aruco440 -lopencv_dnn440

win32:CONFIG(debug, debug|release): LIBS += -LC:\Users\plane\source\opencv-4.4.0\build\install\x86\vc15\lib \
    -lopencv_core440d -lopencv_highgui440d -lopencv_tracking440d -lopencv_video440d -lopencv_imgproc440d -lopencv_videoio440d -lopencv_calib3d440d -lopencv_aruco440d

win32:CONFIG(release, debug|release): LIBS += -LC:\Users\plane\Develop\FlightAgentX\build-uglobalhotkey-Desktop_Qt_5_14_1_MSVC2017_32bit-Release\release\ -lUGlobalHotkey
win32:CONFIG(release, debug|release): LIBS += -LC:\Users\plane\Develop\FlightAgentX\build-uglobalhotkey-Desktop_Qt_5_14_1_MSVC2017_32bit-Debug\debug\ -lUGlobalHotkey


win32:CONFIG(release, debug|release): LIBS += -L"C:\\Program Files (x86)\\YAML_CPP\\lib" -lyaml-cpp
win32:CONFIG(debug, debug|release): LIBS += -L"C:\\Program Files (x86)\\YAML_CPP\\lib" -lyaml-cppd

win32:CONFIG(release, debug|release): LIBS += -L"C:\\Users\\plane\\source\\onnxruntime-win-x86-1.5.1\\lib" -lonnxruntime

SOURCES += \
    src/FSANet.cpp \
    src/HeadPoseDetector.cpp \
    src/KalmanFilter.cpp \
    src/PoseDataSender.cpp \
    src/agentxconfig.cpp \
    src/ekfconfig.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    freetrack/ftnoir_protocol_ft.cpp \
    freetrack/freetrackclient/freetrackclient.c \
    freetrack/shm.cpp \
    freetrack/csv/csv.cpp \
    src/poseremapper.cpp

HEADERS += \
    inc/FSANet.h \
    inc/FlightAgxSettings.h \
    inc/HeadPoseDetector.h \
    inc/KalmanFilter.h \
    inc/PoseDataSender.h \
    inc/agentxconfig.h \
    inc/ekfconfig.h \
    inc/fagx_datatype.h \
    inc/mainwindow.h \
    freetrack/ftnoir_protocol_ft.h \
    freetrack/freetrackclient/fttypes.h \
    freetrack/shm.h \
    freetrack/csv/csv.h \
    inc/poseremapper.h

FORMS += \
    agentxconfig.ui \
    ekfconfig.ui \
    mainwindow.ui

TRANSLATIONS += \
    FlightAgentX_en_US.ts

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    main.qrc
