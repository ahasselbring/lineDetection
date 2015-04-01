TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += main.cpp
QMAKE_CXXFLAGS += -std=c++11


unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv
