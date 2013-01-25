#-------------------------------------------------
#
# Project created by QtCreator 2012-02-11T09:48:46
#
#-------------------------------------------------

QT       += core gui

TARGET = Manytrack
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
          "/usr/lib/libvtkCommon.so.5.8" \
            "/usr/lib/libvtkWidgets.so.5.8.0"





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
          "/usr/lib/libvtkGraphics.so.5.8" \
            "/usr/lib/libvtkWidgets.so.5.8.0"


 DEFINES += EIGEN_DONT_ALIGN_STATICALLY

##use the below command when deploying

  QMAKE_LFLAGS += -Wl,--rpath=\\\$\$ORIGIN/libs
  QMAKE_LFLAGS_RPATH=


#Turn off the messages before you release!
    CONFIG += qt warn_off release
    DEFINES += QT_NO_DEBUG_OUTPUT
    DEFINES += QT_NO_DEBUG

}

macx {

        CONFIG += link_pkgconfig

        PKGCONFIG += opencv
        PKGCONFIG += pcl_io-1.6
		PKGCONFIG += pcl_visualization-1.6
		INCLUDEPATH += "/usr/include/vtk-5.6/"

}

win32-*{


}

SOURCES += main.cpp\
        Manytrack.cpp \
    Track.cpp \
    ICPTracker.cpp \
    icp_color.cpp \
    trans_2D/transformation_estimation_2D.cpp

HEADERS  += \
    Track.h \
    ICPTracker.h \
    Manytrack.h \
    icp_color.hpp \
    icp_color.h \
    trans_2D/transformation_estimation_2D.hpp \
    trans_2D/transformation_estimation_2D.h

FORMS    += Manytrack.ui

RESOURCES += \
    Manytrack.qrc


