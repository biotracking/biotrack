#-------------------------------------------------
#
# Project created by QtCreator 2012-02-29T11:32:05
#
#-------------------------------------------------

QT       += core gui

TARGET = Mugshotter_btf
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    mugshot.cpp \
    paintcanvas.cpp

HEADERS  += mainwindow.h \
    mugshot.h \
    paintcanvas.h


INCLUDEPATH += /usr/include/opencv-2.3.1

LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui


unix {
        CONFIG += link_pkgconfig
        PKGCONFIG += opencv
}

FORMS    += mainwindow.ui
