#-------------------------------------------------
#
# Project created by QtCreator 2012-02-11T09:48:46
#
#-------------------------------------------------

QT       += core gui

TARGET = MultiTrack
TEMPLATE = app





linux-* {
        CONFIG += link_pkgconfig



        PKGCONFIG += opencv
        PKGCONFIG += pcl_io-1.6



#VTK does not have its own PKGCONFIG so we have to manually do this
INCLUDEPATH +=  "/usr/local/include/pcl-1.6/" \
                 "/usr/include/cminpack-1/" \
                 "/usr/include/flann/" \
                 "/usr/include/eigen3/" \
                 "/usr/include/openni/" \
                 "/usr/include/vtk-5.8/" \
          "/usr/lib/libvtkCommon.so.5.8"



LIBS += -lpcl_common \
         -lpcl_io \
         -lpcl_filters \
         -lpcl_visualization \
         -lpcl_registration \
         -lpcl_features \
         -lpcl_segmentation \
         -lpcl_search \
         -lpcl_surface \
         -lpcl_keypoints \
# This stuff is for visualization
          "/usr/lib/libvtkCommon.so.5.8" \
          "/usr/lib/libvtkFiltering.so.5.8" \
          "/usr/lib/libvtkRendering.so.5.8" \
          "/usr/lib/libvtkGraphics.so.5.8"




##use the below command when deploying

#  QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/libs
#  QMAKE_LFLAGS_RPATH=


#Turn off the messages before you release!
#    CONFIG += qt warn_off release
#    DEFINES += QT_NO_DEBUG_OUTPUT
#    DEFINES += QT_NO_DEBUG

}

macx {

        CONFIG += link_pkgconfig

        PKGCONFIG += opencv
        PKGCONFIG += pcl_io-1.6

}

win32-*{


}

SOURCES += main.cpp\
        multitrack.cpp \
    Track.cpp \
    ICPTracker.cpp

HEADERS  += \
    Track.h \
    ICPTracker.h \
    multitrack.h

FORMS    += multitrack.ui

RESOURCES += multitrack.qrc


