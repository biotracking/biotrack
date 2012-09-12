TARGET = abos_optflow
TEMPLATE = app
QT += xml
CONFIG += debug
#CONFIG += release
#DEFINES += DEBUG
unix {
	CONFIG += link_pkgconfig
	PKGCONFIG += opencv
}
SOURCES += main.cpp \
    mainwindow.cpp \
    ofthread.cpp

HEADERS += mainwindow.h \
    ofthread.h

FORMS += mainwindow.ui

INCLUDEPATH += ../../core/
INCLUDEPATH += ../../core/modules/optflow
INCLUDEPATH += ../../core/lib/prosilica
LIBS += -Bstatic ../../core/libabos.a
LIBS += -Bstatic ../../bin/lib/prosilica/libPvAPI.a
