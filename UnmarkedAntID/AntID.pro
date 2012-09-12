#-------------------------------------------------
#
# Project created by QtCreator 2012-08-03T14:44:57
#
#-------------------------------------------------

QT       += core gui

TARGET = AntID
TEMPLATE = app


SOURCES += main.cpp\
        ant_id.cpp


HEADERS  += \
    ant_id.h

    unix {
            CONFIG += link_pkgconfig
            PKGCONFIG += opencv
    }

LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui

FORMS    += ant_id.ui
