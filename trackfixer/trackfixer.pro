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

