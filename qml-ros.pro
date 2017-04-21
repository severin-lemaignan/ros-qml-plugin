#Building of native code

TEMPLATE = lib
TARGET = rosplugin

CONFIG += qt plugin c++11 nostrip link_pkgconfig
CONFIG += debug
#CONFIG -= android_install
PKGCONFIG += roscpp tf image_transport

QT += qml quick widgets

TARGET = $$qtLibraryTarget($$TARGET)
uri = Ros

HEADERS += \
    src/RosPlugin.h \
    src/RosImageProvider.h \
    src/ros.h

SOURCES += \
    src/RosPlugin.cpp \
    src/RosImageProvider.cpp \
    src/ros.cpp

#File installation

qmldir.files = qmldir
javascript.files = src/ros-toolkit.js
qml.files = src/Ros.qml

OTHER_FILES += qmldir.files javascript.files qml.files

unix {
    installPath = $$[QT_INSTALL_QML]/$$replace(uri, \\., /)
    qmldir.path = $$installPath
    javascript.path = $$installPath
    qml.path = $$installPath
    target.path = $$installPath
    INSTALLS += target qmldir javascript qml
}

DISTFILES += \
    README.md

