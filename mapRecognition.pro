TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += \
    src/griddetect.cpp \
    src/main.cpp \
    src/transformer.cpp \
    src/findrect.cpp \
    src/leddetect.cpp \
    src/base.cpp \
    src/gridfourdetect.cpp \
    src/markerdetect.cpp

INCLUDEPATH += /usr/local/include/opencv \
/usr/local/include/opencv2 \
/usr/local/include \
/home/gogojjh/QT/mapRecognition/include

LIBS += /usr/local/lib/libopencv_core.so.2.4
LIBS += /usr/local/lib/libopencv_highgui.so.2.4
LIBS += /usr/local/lib/libopencv_calib3d.so.2.4
LIBS += /usr/local/lib/libopencv_gpu.so.2.4
LIBS += /usr/local/lib/libopencv_imgproc.so.2.4
LIBS += /usr/local/lib/libopencv_legacy.so.2.4
LIBS += /usr/local/lib/libopencv_ml.so.2.4
LIBS += /usr/local/lib/libopencv_objdetect.so.2.4
LIBS += /usr/local/lib/libopencv_ocl.so.2.4
LIBS += /usr/local/lib/libopencv_contrib.so.2.4
LIBS += /usr/local/lib/libopencv_core.so.2.4
LIBS += /usr/local/lib/libopencv_features2d.so.2.4
LIBS += /usr/local/lib/libopencv_flann.so.2.4
LIBS += /usr/local/lib/libopencv_photo.so.2.4
LIBS += /usr/local/lib/libopencv_stitching.so.2.4
LIBS += /usr/local/lib/libopencv_superres.so.2.4
LIBS += /usr/local/lib/libopencv_video.so.2.4
LIBS += /usr/local/lib/libopencv_videostab.so.2.4
LIBS += /usr/local/lib/libopencv_contrib.so.2.4
LIBS += /usr/local/lib/libopencv_core.so.2.4
LIBS += /usr/local/lib/libopencv_features2d.so.2.4

HEADERS += \
    include/maprecognition/base.h \
    include/maprecognition/griddetect.h \
    include/maprecognition/histogram.h \
    include/maprecognition/transformer.h \
    include/maprecognition/findrect.h \
    include/maprecognition/leddetect.h \
    include/maprecognition/gridfourdetect.h \
    include/maprecognition/markerdetect.h
