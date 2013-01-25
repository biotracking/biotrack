TARGET = trackfixer
TEMPLATE = app
QT += xml
CONFIG += debug
unix {
	CONFIG += link_pkgconfig
	PKGCONFIG += opencv
}
SOURCES += main.cpp \
    mainwindow.cpp    

HEADERS += mainwindow.h   

FORMS += mainwindow.ui

##use the below command when deploying

  QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/libs
  QMAKE_LFLAGS_RPATH=


#Turn off the messages before you release!
    CONFIG += qt warn_off release
    DEFINES += QT_NO_DEBUG_OUTPUT
    DEFINES += QT_NO_DEBUG
