TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += main.cpp


unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += opencv
