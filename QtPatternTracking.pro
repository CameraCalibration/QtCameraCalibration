QT += core gui widgets

TARGET = PatternTracking
TEMPLATE = app
CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp \
    mainwindow.cpp \
    patterndetector.cpp \
#    trackinggrid.cpp \
    cameracalibrator.cpp

OPENCV_DIR = "D:\opt\opencv\build"

INCLUDEPATH += $$OPENCV_DIR/include
QMAKE_LIBDIR += $$OPENCV_DIR/x64/vc15/lib

CONFIG(debug, debug|release) {
    LIBS += -lopencv_world344d
}
else {
    LIBS += -lopencv_world344
}

HEADERS += \
    mainwindow.h \
    patterndetector.h \
#    trackinggrid.h \
    cameracalibrator.h \
    image.h \
    geometria.h \
    constants.h \
    metrics.h
