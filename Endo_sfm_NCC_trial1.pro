#-------------------------------------------------
#
# Project created by QtCreator 2017-06-24T17:46:46
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Endo_sfm_NCC_trial1
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS




INCLUDEPATH += /home/terminalx/libs/opencv-3.2.0/modules/core/include\
/home/terminalx/libs/opencv-3.2.0/modules/imgcodecs/include\
/home/terminalx/libs/opencv-3.2.0/modules/highgui/include\
/home/terminalx/libs/opencv-3.2.0/modules/imgproc/include\
/home/terminalx/libs/opencv-3.2.0/modules/highgui/include\
/usr/local/include\
/home/terminalx/libs/opencv_contrib-master/modules/sfm/include\
/home/terminalx/libs/opencv-3.2.0/modules/viz/include\
/home/terminalx/libs/opencv-3.2.0/modules/videoio/include\
/home/terminalx/libs/opencv-3.2.0/modules/video/include\
/home/terminalx/libs/opencv_contrib-master/modules/xfeatures2d/include\
/home/terminalx/libs/opencv-3.2.0/modules/features2d/include\

LIBS += -L/home/terminalx/libs/opencv-3.2.0/build/lib

LIBS     += -lopencv_core -lopencv_highgui -lopencv_video -lopencv_imgcodecs -lopencv_sfm -lopencv_calib3d -lopencv_videoio -lopencv_viz -lopencv_imgproc -lopencv_features2d -lopencv_xfeatures2d -lopencv_video

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui
