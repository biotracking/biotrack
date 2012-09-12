TEMPLATE = app
TARGET = ParkingSpotOverlay 

QT        += core 

HEADERS   +=
SOURCES   += main.cpp
FORMS	  +=
RESOURCES +=
LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc
