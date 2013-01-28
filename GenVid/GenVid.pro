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



#Turn off the messages before you release!
    CONFIG += qt warn_off release
    DEFINES += QT_NO_DEBUG_OUTPUT
    DEFINES += QT_NO_DEBUG

linux-*{
        CONFIG += link_pkgconfig
#        PKGCONFIG += opencv
		INCLUDEPATH += /usr/include/opencv
		# INCLUDEPATH += /usr/include/opencv-2.3.1
		LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui

##use the below command when deploying

  QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/libs
  QMAKE_LFLAGS_RPATH=
}

macx {
	CONFIG += link_pkgconfig
	PKGCONFIG += opencv
	ICON = $${PWD}/res/genicons.icns
}

RESOURCES += \
    resources.qrc

