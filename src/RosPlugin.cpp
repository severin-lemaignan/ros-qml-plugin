/**
 * @file RosPlugin.cpp
 * @brief Source for exporting QML objects pose as ROS TF frames
 * @author Séverin Lemaignan
 * @version 1.0
 * @date 2016-11-29
 */

#include <vector>
#include <string>

#include<QQmlEngine>
#include <ros/ros.h>

#include "RosPlugin.h"

void RosPlugin::registerTypes(const char *uri){
    qmlRegisterType<RosNode>(uri, 1, 0, "RosNode");
    qmlRegisterType<TFBroadcaster>(uri, 1, 0, "TFBroadcaster");
}

void RosPlugin::initializeEngine(QQmlEngine *engine, const char *uri){

    auto remappings = std::vector<std::pair<std::string, std::string>>();
    ros::init(remappings,"sandtray");

}

