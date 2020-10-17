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
INCLUDEPATH += ../third_party/opencv-4.4.0-build/include
INCLUDEPATH +=../third_party/eigen-3.3.7
INCLUDEPATH += ../third_party/dlib/include
INCLUDEPATH += ../third_party/YAML_CPP/include
INCLUDEPATH += ./inc/
INCLUDEPATH += ../third_party/onnxruntime-win-x86-1.5.1/include
CONFIG += force_debug_info
RC_ICONS = icon.ico

include(../third_party/QJoysticks/QJoysticks.pri)
include(../third_party/UGlobalHotkey/UGlobalHotkey.pri)

contains(QT_ARCH, i386) {
    win32:CONFIG(release, debug|release): LIBS += ../third_party/dlib/lib/dlib19.21.0_release_32bit_msvc1916.lib

    win32:CONFIG(release, debug|release): LIBS += -L../third_party/opencv-4.4.0-build/x86/vc15/lib \
        -lopencv_core440 -lopencv_highgui440 -lopencv_tracking440 -lopencv_video440 -lopencv_imgproc440 -lopencv_videoio440 -lopencv_calib3d440 -lopencv_aruco440 -lopencv_dnn440

    win32:CONFIG(debug, debug|release): LIBS += -L../third_party/opencv-4.4.0-build/x86/vc15/lib \
        -lopencv_core440d -lopencv_highgui440d -lopencv_tracking440d -lopencv_video440d -lopencv_imgproc440d -lopencv_videoio440d -lopencv_calib3d440d -lopencv_aruco440d

    win32:CONFIG(release, debug|release): LIBS += -L"../third_party/YAML_CPP_x86/lib" -lyaml-cpp
    win32:CONFIG(debug, debug|release): LIBS += -L"../third_party/YAML_CPP_x86/lib" -lyaml-cppd

    win32:CONFIG(release, debug|release): LIBS += -L../third_party/onnxruntime-win-x86-1.5.1/lib -lonnxruntime
    win32:CONFIG(release, debug|release): LIBS += -L../third_party/OpenBLAS-0.3.10-x86/lib -llibopenblas
} else {

win32:CONFIG(debug, debug|release): LIBS += ../third_party/dlib/lib/dlib19.21.0_debug_64bit_msvc1916.lib
win32:CONFIG(release, debug|release): LIBS += ../third_party/dlib/lib/dlib19.21.0_release_64bit_msvc1916.lib

win32:CONFIG(release, debug|release): LIBS += -L../third_party/opencv-4.4.0-build/x64/vc15/lib \
    -lopencv_core440 -lopencv_highgui440 -lopencv_tracking440 -lopencv_video440 -lopencv_imgproc440 -lopencv_videoio440 -lopencv_calib3d440 -lopencv_aruco440 -lopencv_dnn440

win32:CONFIG(debug, debug|release): LIBS += -L../third_party/opencv-4.4.0-build/x64/vc15/lib \
    -lopencv_core440d -lopencv_highgui440d -lopencv_tracking440d -lopencv_video440d -lopencv_imgproc440d -lopencv_videoio440d -lopencv_calib3d440d -lopencv_aruco440d

win32:CONFIG(release, debug|release): LIBS += -L"../third_party/YAML_CPP/lib" -lyaml-cpp
win32:CONFIG(debug, debug|release): LIBS += -L"../third_party/YAML_CPP/lib" -lyaml-cppd

win32:CONFIG(release, debug|release): LIBS += -L../third_party/onnxruntime-win-x64-1.5.1/lib -lonnxruntime
win32:CONFIG(release, debug|release): LIBS += -L../third_party/OpenBLAS-0.3.10-x64/lib -llibopenblas
}

SOURCES += \
    src/FSANet.cpp \
    src/FaceDetectors.cpp \
    src/FlightAgxSettings.cpp \
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
    inc/FaceDetectors.h \
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

DISTFILES +=
