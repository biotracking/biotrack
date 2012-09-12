TARGET = abos_aquarium
TEMPLATE = app
QT += xml
Debug:CONFIG += debug
unix {
	CONFIG += link_pkgconfig
	PKGCONFIG += opencv
}
SOURCES += src/main.cpp \
    src/mainwindow.cpp \
	src/ofthread.cpp \
    src/sliders_thread.cpp \
    src/blobs.cpp \
    src/Blob.cpp \
    src/BlobPairing.cpp

HEADERS += src/mainwindow.h \
	src/ofthread.h \
    src/sliders_thread.h \
    src/blobs.h \
    src/Blob.h \
    src/BlobPairing.h

FORMS += forms/mainwindow.ui

INCLUDEPATH += ../../core/
INCLUDEPATH += ../../core/modules/optflow
INCLUDEPATH += ../../core/lib/prosilica
INCLUDEPATH += /usr/include/libxml2

LIBS += -Bstatic ../../core/libabos.a

HARDWARE_PLATFORM = $$system(uname -i)
contains( HARDWARE_PLATFORM, x86_64 ) {
    LIBS += -Bstatic ../../bin/lib/prosilica/x64/libPvAPI.a
} else {
    LIBS += -Bstatic ../../bin/lib/prosilica/x86/libPvAPI.a
}


LIBS += -lxml2 /usr/local/lib/libopencv_calib3d.so /usr/local/lib/libopencv_contrib.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_features2d.so /usr/local/lib/libopencv_flann.so /usr/local/lib/libopencv_gpu.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgproc.so /usr/local/lib/libopencv_legacy.so /usr/local/lib/libopencv_ml.so /usr/local/lib/libopencv_nonfree.so /usr/local/lib/libopencv_objdetect.so /usr/local/lib/libopencv_photo.so /usr/local/lib/libopencv_stitching.so /usr/local/lib/libopencv_ts.so /usr/local/lib/libopencv_video.so /usr/local/lib/libopencv_videostab.so

QMAKE_CXXFLAGS_RELEASE += -O3
