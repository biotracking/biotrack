#-------------------------------------------------
#
# Project created by QtCreator 2012-05-13T20:05:12
#
#-------------------------------------------------

QT       += core gui

TARGET = Backgrounder
TEMPLATE = app

##
#   Import Libraries
#
##

QMAKE_LFLAGS += -static-libgcc

linux-* {
        CONFIG += link_pkgconfig
        PKGCONFIG += opencv

##use the below command when deploying

  QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/libs
  QMAKE_LFLAGS_RPATH=


}
#Turn off the messages before you release!
    CONFIG += qt warn_off release
    DEFINES += QT_NO_DEBUG_OUTPUT
    DEFINES += QT_NO_DEBUG

macx {

        CONFIG += link_pkgconfig
        PKGCONFIG += opencv
}






SOURCES += main.cpp\
        backgrounder.cpp \
    ModeAccumulator.cpp \
#    BackgroundToolApp.cpp \
    BackgroundCalculatorMode.cpp \
    BackgroundCalculatorAverage.cpp \
    BackgroundCalculator.cpp \
#    bgcalc.cpp

HEADERS  += backgrounder.h \
   ModeAccumulator.h \
#    BackgroundToolApp.h \
    BackgroundCalculatorMode.h \
    BackgroundCalculatorAverage.h \
 BackgroundCalculator.h \
#    bgcalc.h

FORMS    += backgrounder.ui

OTHER_FILES += \
    Backgrounder.pro.user

RESOURCES += \
    Backgrounder.qrc
