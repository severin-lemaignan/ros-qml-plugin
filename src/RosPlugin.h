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


#include <ros/spinner.h>
#include "ros.h"

/**
 * @brief A ROS bridge for QML objects
 *
 * Attention, the QML application is responsible to call ros::init().
 */
class RosPlugin : public QQmlExtensionPlugin
{
Q_OBJECT
    Q_PLUGIN_METADATA(IID "org.qt-project.Qt.QQmlExtensionInterface")

public:
    RosPlugin():spinner(1) {}

    void registerTypes(const char *uri);
    void initializeEngine(QQmlEngine *engine, const char *uri);

private:

    ros::AsyncSpinner spinner;
};

#endif // ROSPLUGIN_H
