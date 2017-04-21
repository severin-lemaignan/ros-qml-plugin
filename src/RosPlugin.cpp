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

#include <QQmlEngine>
#include <ros/ros.h>

#include "RosPlugin.h"

#include "RosImageProvider.h"

void RosPlugin::registerTypes(const char *uri){
    qmlRegisterType<RosPoseSubscriber>(uri, 1, 0, "RosPoseSubscriber");
    qmlRegisterType<RosPosePublisher>(uri, 1, 0, "RosPosePublisher");
    qmlRegisterType<RosStringSubscriber>(uri, 1, 0, "RosStringSubscriber");
    qmlRegisterType<RosStringPublisher>(uri, 1, 0, "RosStringPublisher");
    qmlRegisterType<TFListener>(uri, 1, 0, "TFListener");
    qmlRegisterType<TFBroadcaster>(uri, 1, 0, "TFBroadcaster");
    qmlRegisterType<FootprintsPublisher>(uri, 1, 0, "FootprintsPublisher");
    qmlRegisterType<RosSignal>(uri, 1, 0, "RosSignal");
    qmlRegisterType<ImagePublisher>(uri, 1, 0, "ImagePublisher");
}

void RosPlugin::initializeEngine(QQmlEngine *engine, const char *uri){

    Q_UNUSED(uri);

    spinner.start();
    engine->addImageProvider("rosimage", new RosImageProvider);
}

