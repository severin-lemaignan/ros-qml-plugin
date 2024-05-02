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
#include "ros_qml_plugin/qml_rosparam.hpp"
#include "ros_qml_plugin/qml_rossignal.hpp"
#include "ros_qml_plugin/qml_rostopic.hpp"
#include "ros_qml_plugin/ros2.hpp"

#include <hri_msgs/msg/expression.hpp>
#include <hri_msgs/msg/live_speech.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>

class RosPlugin : public QQmlExtensionPlugin {
  Q_OBJECT
  Q_PLUGIN_METADATA(IID QQmlExtensionInterface_iid)

public:
  void registerTypes(const char *uri) override {
    Q_ASSERT(uri == QLatin1String("Ros"));

    qmlRegisterType<RosParam>(uri, 2, 0, "RosParam");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::String>>(uri, 2, 0,
                                                         "StringTopic");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::Int16>>(uri, 2, 0, "IntTopic");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::Float32>>(uri, 2, 0,
                                                          "FloatTopic");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::Bool>>(uri, 2, 0, "BoolTopic");
    qmlRegisterType<RosTopicImpl<hri_msgs::msg::Expression>>(uri, 2, 0,
                                                             "ExpressionTopic");
    qmlRegisterType<RosTopicImpl<hri_msgs::msg::LiveSpeech>>(uri, 2, 0,
                                                             "LiveSpeechTopic");

    qmlRegisterType<RosSignal>(uri, 2, 0, "RosSignal");
  }

  void initializeEngine(QQmlEngine *engine, const char *uri) {
    Q_UNUSED(uri);

    std::cout << "Initializing the ROS 2 node" << std::endl;
    Ros2Qml::getInstance().init("qml_ros2_node");

    engine->addImageProvider("rosimage", new RosImageProvider);
  }
};

#include "rosplugin.moc"
