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

#include "ros_qml_plugin/ros.hpp"
#include "ros_qml_plugin/ros_types.hpp"
#include "image_provider.hpp"
#include "ros_qml_plugin/qml_rosparam.hpp"
#include "ros_qml_plugin/qml_rossignal.hpp"
#include "ros_qml_plugin/qml_rostopic.hpp"
#include "ros_qml_plugin/qml_rosservice.hpp"
#include "ros_qml_plugin/qml_rosaction.hpp"
#include "ros_qml_plugin/ros2.hpp"

#include <hri_msgs/msg/expression.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros_qml_plugin/qml_sayskill.hpp"
#include "ros_qml_plugin/qml_chatskill.hpp"
#include "ros_qml_plugin/qml_set_expression_skill.hpp"
#include "ros_qml_plugin/qml_look_at_skill.hpp"
#include "ros_qml_plugin/qml_navigate_skill.hpp"

class RosPlugin : public QQmlExtensionPlugin
{
  Q_OBJECT
  Q_PLUGIN_METADATA(IID QQmlExtensionInterface_iid)

public:
  void registerTypes(const char * uri) override
  {
    Q_ASSERT(uri == QLatin1String("Ros"));

    qRegisterMetaType<RosPoint>("RosPoint");
    qRegisterMetaType<RosPose>("RosPose");

    qmlRegisterSingletonType<Ros>(
      uri, 2, 0, "Ros", [](QQmlEngine * engine, QJSEngine *) -> QObject * {
        Q_UNUSED(engine);
        return new Ros();
      });

    qmlRegisterType<RosParam>(uri, 2, 0, "RosParam");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::String>>(
      uri, 2, 0,
      "StringTopic");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::Int16>>(uri, 2, 0, "IntTopic");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::Int32>>(uri, 2, 0, "Int32Topic");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::Float32>>(
      uri, 2, 0,
      "FloatTopic");
    qmlRegisterType<RosTopicImpl<std_msgs::msg::Bool>>(uri, 2, 0, "BoolTopic");
    qmlRegisterType<RosTopicImpl<hri_msgs::msg::Expression>>(
      uri, 2, 0,
      "ExpressionTopic");
    qmlRegisterType<ClosedCaptionTopic>(uri, 2, 0, "ClosedCaptionTopic");
    qmlRegisterType<LiveSpeechTopic>(uri, 2, 0, "LiveSpeechTopic");
    qmlRegisterType<IntentTopic>(uri, 2, 0, "IntentTopic");

    qmlRegisterType<RosSignal>(uri, 2, 0, "RosSignal");

    qmlRegisterType<SetBoolService>(uri, 2, 0, "SetBoolService");

    qmlRegisterType<SetUiFragmentService>(uri, 2, 0, "SetUiFragmentService");

    qmlRegisterType<GetLocalesService>(uri, 2, 0, "GetLocalesService");

    qmlRegisterType<SetLocaleAction>(uri, 2, 0, "SetLocaleAction");

    qmlRegisterType<SaySkill>(uri, 2, 0, "SaySkill");
    qmlRegisterType<ChatSkill>(uri, 2, 0, "ChatSkill");
    qmlRegisterType<SetExpressionSkill>(uri, 2, 0, "SetExpressionSkill");
    qmlRegisterType<LookAtSkill>(uri, 2, 0, "LookAtSkill");
    qmlRegisterType<NavigateSkill>(uri, 2, 0, "NavigateSkill");
  }

  void initializeEngine(QQmlEngine * engine, const char * uri)
  {
    Q_UNUSED(uri);

    QString rosNodeName("qml_ros2_node");

    std::cout << "Initializing the ROS 2 node" << std::endl;

    QVariant param = engine->rootContext()->contextProperty("ROSNodeName");
    if (param.isValid()) {
      rosNodeName = param.toString();
      std::cout << "Node name: " << rosNodeName.toStdString() << std::endl;
    } else {
      std::cout << "No ROS node name provided, using default: " << rosNodeName.toStdString()
                << std::endl;
    }

    Ros2Qml::getInstance().init(rosNodeName);

    engine->addImageProvider("rosimage", new RosImageProvider);
  }
};

#include "rosplugin.moc"
