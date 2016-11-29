/**
 * @file RosPlugin.cpp
 * @brief Source for exporting QML objects pose as ROS TF frames
 * @author Séverin Lemaignan
 * @version 1.0
 * @date 2016-11-29
 */

#include "RosPlugin.h"

#include<QQmlEngine>

void RosPlugin::registerTypes(const char *uri){
    qmlRegisterType<Ros>(uri, 1, 0, "Ros");
}

void RosPlugin::initializeEngine(QQmlEngine *engine, const char *uri){
}

