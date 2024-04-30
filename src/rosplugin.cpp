// Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QQmlEngine>
#include <QQmlExtensionPlugin>
#include <QtQml>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "image_provider.hpp"
#include "ros_qml_plugin/qmlobjects.hpp"
#include "ros_qml_plugin/ros2.hpp"

class RosPlugin : public QQmlExtensionPlugin {
  Q_OBJECT
  Q_PLUGIN_METADATA(IID QQmlExtensionInterface_iid)

public:
  void registerTypes(const char *uri) override {
    Q_ASSERT(uri == QLatin1String("Ros"));

    //  qmlRegisterType<RosPoseSubscriber>(uri, 2, 0, "RosPoseSubscriber");
    //  qmlRegisterType<RosPosePublisher>(uri, 2, 0, "RosPosePublisher");
    qmlRegisterType<RosParam>(uri, 2, 0, "RosParam");
    qmlRegisterType<RosStringSubscriber>(uri, 2, 0, "RosStringSubscriber");
    qmlRegisterType<RosStringPublisher>(uri, 2, 0, "RosStringPublisher");
    //  qmlRegisterType<TFListener>(uri, 2, 0, "TFListener");
    //  qmlRegisterType<TFBroadcaster>(uri, 2, 0, "TFBroadcaster");
    //  qmlRegisterType<FootprintsPublisher>(uri, 2, 0, "FootprintsPublisher");
    qmlRegisterType<RosSignal>(uri, 2, 0, "RosSignal");
    //  qmlRegisterType<ImagePublisher>(uri, 2, 0, "ImagePublisher");
  }

  void initializeEngine(QQmlEngine *engine, const char *uri) {
    Q_UNUSED(uri);

    std::cout << "Initializing the ROS 2 node" << std::endl;
    Ros2Qml::getInstance().init("qml_ros2_node");

    engine->addImageProvider("rosimage", new RosImageProvider);
  }
};

#include "rosplugin.moc"
