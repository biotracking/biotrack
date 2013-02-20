TEMPLATE = app
TARGET = ParkingSpotOverlay 

QT        += core 
CONFIG    += debug

HEADERS   +=
SOURCES   += main.cpp
FORMS	  +=
RESOURCES +=
LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc
