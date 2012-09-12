    #-------------------------------------------------
#
# Project created by QtCreator 2012-05-18T19:30:12
#
#-------------------------------------------------

QT       += core gui

TARGET = GenVid
TEMPLATE = app


SOURCES += main.cpp\
        genvid.cpp

HEADERS  += genvid.h

FORMS    += genvid.ui

INCLUDEPATH += /usr/include/opencv

# INCLUDEPATH += /usr/include/opencv-2.3.1

LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui

unix {
        CONFIG += link_pkgconfig
#        PKGCONFIG += opencv
}

RESOURCES += \
    resources.qrc

