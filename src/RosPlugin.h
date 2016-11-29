/**
 * @file RosPlugin.cpp
 * @brief Header for exporting QML objects pose as ROS TF frames
 * @author Séverin Lemaignan
 * @version 1.0
 * @date 2016-11-29
 */

#ifndef ROSPLUGIN_H
#define ROSPLUGIN_H

#include <QQmlExtensionPlugin>
#include <qqml.h>

#include "ros.h"

/**
 * @brief Object that exposes export the target QML item as ROS TF frames
 */
class RosPlugin : public QQmlExtensionPlugin
{
Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QQmlExtensionInterface")

public:
    void registerTypes(const char *uri);
    void initializeEngine(QQmlEngine *engine, const char *uri);
};

#endif // ROSPLUGIN_H
