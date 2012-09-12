TARGET = abos_tutorial
TEMPLATE = app
QT += xml
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
INCLUDEPATH += ../../core/lib/prosilica
LIBS += -Bstatic ../../core/libabos.a
LIBS += -Bstatic ../../bin/lib/prosilica/libPvAPI.a
