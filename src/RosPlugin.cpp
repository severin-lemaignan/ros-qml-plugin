/**
 * @file RosPlugin.cpp
 * @brief Source for exporting QML objects pose as ROS TF frames
 * @author Séverin Lemaignan
 * @version 1.0
 * @date 2016-11-29
 */

#include <vector>
#include <string>
#include <thread>

#include<QQmlEngine>
#include <ros/ros.h>

#include "RosPlugin.h"

void RosPlugin::registerTypes(const char *uri){
    qmlRegisterType<RosPositionController>(uri, 1, 0, "RosPositionController");
    qmlRegisterType<TFBroadcaster>(uri, 1, 0, "TFBroadcaster");
}

void RosPlugin::initializeEngine(QQmlEngine *engine, const char *uri){


    spinner.start();
}

