QT       += core gui network charts
QT += widgets

CONFIG += c++17
DEFINES += QT_DEPRECATED_WARNINGS _USE_MATH_DEFINES _MBCS CERES_MSVC_USE_UNDERSCORE_PREFIXED_BESSEL_FUNCTIONS GOOGLE_GLOG_DLL_DECL=
QMAKE_CXXFLAGS *= /std:c++17
INCLUDEPATH += ../third_party/opencv-4.4.0-build/include
#INCLUDEPATH +=../third_party/eigen-3.3.7
INCLUDEPATH += ../third_party/dlib/include
INCLUDEPATH += ../third_party/YAML_CPP/include
INCLUDEPATH += ./inc/
INCLUDEPATH += ./lib/
INCLUDEPATH += ./lib/accela_filter/
INCLUDEPATH += ../third_party/libusb-1.0.23/include/libusb-1.0

INCLUDEPATH += ../third_party/onnxruntime-win-x64-gpu-1.5.2/include
INCLUDEPATH += ../third_party/ceres-windows/ceres-solver/include/
INCLUDEPATH += ../third_party/ceres-windows/win/include
INCLUDEPATH += ../third_party/ceres-windows/glog/src/windows
INCLUDEPATH += ../third_party/ceres-windows/Eigen


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

    win32:CONFIG(release, debug|release): LIBS += -L../third_party/libusb-1.0.23/MS32/static -llibusb-1.0
} else {
    win32:CONFIG(debug, debug|release): LIBS += ../third_party/dlib/lib/dlib19.21.0_debug_64bit_msvc1916.lib
    win32:CONFIG(release, debug|release): LIBS += ../third_party/dlib/lib/dlib19.21.0_release_64bit_msvc1916.lib

    win32:CONFIG(release, debug|release): LIBS += -L../third_party/opencv-4.4.0-build/x64/vc15/lib \
        -lopencv_core440 -lopencv_highgui440 -lopencv_tracking440 -lopencv_video440 -lopencv_imgproc440 -lopencv_videoio440 -lopencv_calib3d440 -lopencv_aruco440 -lopencv_dnn440

    win32:CONFIG(debug, debug|release): LIBS += -L../third_party/opencv-4.4.0-build/x64/vc15/lib \
        -lopencv_core440d -lopencv_highgui440d -lopencv_tracking440d -lopencv_video440d -lopencv_imgproc440d -lopencv_videoio440d -lopencv_calib3d440d -lopencv_aruco440d

    win32:CONFIG(release, debug|release): LIBS += -L"../third_party/YAML_CPP/lib" -lyaml-cpp
    win32:CONFIG(debug, debug|release): LIBS += -L"../third_party/YAML_CPP/lib" -lyaml-cppd

    win32:CONFIG(release, debug|release): LIBS += -L../third_party/onnxruntime-win-x64-gpu-trt-1.5.2/lib -lonnxruntime
    win32:CONFIG(release, debug|release): LIBS += -L../third_party/OpenBLAS-0.3.10-x64/lib -llibopenblas
    win32:CONFIG(release, debug|release): LIBS += -L../third_party/libusb-1.0.23/X64-VS2017 -llibusb-1.0
    win32:CONFIG(release, debug|release): LIBS += -L../third_party/ceres-windows/x64/Release -lceres -llibglog_static
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
    src/filterconfig.cpp \
    src/main.cpp \
    src/mainwindow.cpp \
    lib/freetrack/ftnoir_protocol_ft.cpp \
    lib/freetrack/freetrackclient/freetrackclient.c \
    lib/freetrack/shm.cpp \
    lib/freetrack/csv/csv.cpp \
    src/math_tests.cpp \
    src/stereo_bundle_adjustment.cpp \ 
    src/poseremapper.cpp \
    lib/PS3EYEDriver/src/ps3eye.cpp \
    lib/PS3EYEDriver/src/ps3eye_capi.cpp \
    lib/accela_filter/filter_accela.cpp

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
    inc/filterconfig.h \
    inc/mainwindow.h \
    inc/utils.h \
    inc/stereo_bundle_adjustment.h \
    inc/reprojection_error.h \
    lib/freetrack/ftnoir_protocol_ft.h \
    lib/freetrack/freetrackclient/fttypes.h \
    lib/freetrack/shm.h \
    lib/freetrack/csv/csv.h \
    inc/poseremapper.h \
    lib/accela_filter/filter_accela.h \
    lib/accela_filter/accela-settings.hpp

FORMS += \
    agentxconfig.ui \
    ekfconfig.ui \
    filterconfig.ui \
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
