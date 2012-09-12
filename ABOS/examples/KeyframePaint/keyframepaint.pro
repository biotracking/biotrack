TARGET = keyframepainter
TEMPLATE = app
QT += xml
CONFIG += debug
unix {
	CONFIG += link_pkgconfig
	PKGCONFIG += opencv
}
SOURCES += main.cpp \
    paintcanvas.cpp \
    keyframepaint.cpp

HEADERS += \
    paintcanvas.h \
    keyframepaint.h

FORMS += mainwindow.ui

INCLUDEPATH += ../../core/
INCLUDEPATH += ../../core/lib/prosilica
LIBS += -Bstatic ../../core/libabos.a
LIBS += -Bstatic ../../bin/lib/prosilica/libPvAPI.a


















