TEMPLATE = app
TARGET = AntHistogramsWithKNN
QT += core \
    xml
HEADERS += main.h
SOURCES += main.cpp
FORMS += 
RESOURCES += 
LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui
