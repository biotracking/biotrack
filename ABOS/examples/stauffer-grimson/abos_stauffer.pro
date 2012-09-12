TARGET = abos_stauffer
TEMPLATE = app
QT += xml
#CONFIG += debug
CONFIG += release
#DEFINES += DEBUG
unix {
	CONFIG += link_pkgconfig
	PKGCONFIG += opencv
}
SOURCES += main.cpp \
    mainwindow.cpp \
	background.cpp 

HEADERS += mainwindow.h \
	background.h 

FORMS += mainwindow.ui

INCLUDEPATH += ../../core/
INCLUDEPATH += ../../core/modules/stauffer-grimson
INCLUDEPATH += ../../core/lib/prosilica
LIBS += -Bstatic ../../core/libabos.a
LIBS += -Bstatic ../../bin/lib/prosilica/libPvAPI.a
