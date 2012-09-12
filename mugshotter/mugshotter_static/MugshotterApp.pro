#-------------------------------------------------
#
# Project created by QtCreator 2012-02-29T11:32:05
#
#-------------------------------------------------




QT       += core gui

TARGET = MugshotterApp
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    mugshot.cpp \
    paintcanvas.cpp

HEADERS  += mainwindow.h \
    mugshot.h \
    paintcanvas.h


INCLUDEPATH += /usr/include/opencv

LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui

unix {
        CONFIG += link_pkgconfig
        PKGCONFIG += opencv
}

FORMS    += mainwindow.ui

CONFIG += debug
#CONFIG += qt warn_off release
#DEFINES += QT_NO_DEBUG_OUTPUT
#DEFINES += QT_NO_DEBUG
