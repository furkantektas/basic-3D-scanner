#-------------------------------------------------
#
# Project created by QtCreator 2013-12-14T22:27:18
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = cse-395-g7-ui
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

LIBS += -L/usr/lib/vtk #folder to your VTK library
LIBS += -lvtkCommon \
        -lvtkRendering \
        -lvtkVolumeRendering \
        -lQVTK \
        -lvtkIO \
        -lvtkFiltering \
        -lvtkgdcm

INCLUDEPATH += /usr/include/vtk #folder to your VTK include files

RESOURCES += \
    resources.qrc