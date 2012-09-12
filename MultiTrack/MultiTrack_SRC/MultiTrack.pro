#-------------------------------------------------
#
# Project created by QtCreator 2012-02-11T09:48:46
#
#-------------------------------------------------

QT       += core gui

TARGET = MultiTrack
TEMPLATE = app





linux-* {
        CONFIG += link_pkgconfig

        PKGCONFIG += opencv
        PKGCONFIG += pcl_io-1.6
#       LIBS+= lboost_system

#use the below command when deploying

  QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/libs
  QMAKE_LFLAGS_RPATH=


#Turn off the messages before you release!
#    CONFIG += qt warn_off release
#    DEFINES += QT_NO_DEBUG_OUTPUT
#    DEFINES += QT_NO_DEBUG

}

macx {

        CONFIG += link_pkgconfig

        PKGCONFIG += opencv
        PKGCONFIG += pcl_io-1.6

}

win32-*{


}

SOURCES += main.cpp\
        multitrack.cpp \
    Track.cpp \
    ICPTracker.cpp

HEADERS  += \
    Track.h \
    ICPTracker.h \
    multitrack.h


FORMS    += multitrack.ui

RESOURCES += multitrack.qrc


