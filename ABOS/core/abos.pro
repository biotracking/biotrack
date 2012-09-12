# -------------------------------------------------
# Project created by QtCreator 2010-03-23T22:51:49
# -------------------------------------------------
TARGET = abos
TEMPLATE = lib
QT += xml
#CONFIG += debug static
CONFIG += release static
unix {
	CONFIG += link_pkgconfig
	PKGCONFIG += opencv
}
SOURCES += AbosThread.cpp \
    prosilica/camera.cpp \
    prosilica/Image.cpp \
    prosilicacapture.cpp \
    imagecapture.cpp \
	poolframe.cpp \
	abospool.cpp \ 
    modules/stauffer-grimson/GaussianDist.cpp \
    modules/stauffer-grimson/MoG.cpp \
    modules/optflow/optFlow.cpp 
HEADERS += AbosThread.h \
    prosilica/ImageSource.h \
    prosilica/camera.h \
    prosilica/Image.h \
    prosilicacapture.h \
    imagecapture.h \
	poolframe.h \
	abospool.h \
    modules/stauffer-grimson/GaussianDist.h \
    modules/stauffer-grimson/MoG.h \
    modules/optflow/optFlow.h 

contains( HARDWARE_PLATFORM, x86_64 ) {
    LIBS += -Bstatic ../bin/lib/prosilica/x64/libPvAPI.a
} else {
    LIBS += -Bstatic ../bin/lib/prosilica/x86/libPvAPI.a
}

INCLUDEPATH += ./lib/prosilica
INCLUDEPATH += ./prosilica
