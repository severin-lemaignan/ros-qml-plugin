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

#ifndef ROS_QML_PLUGIN__QML_ROSSIGNAL_HPP_
#define ROS_QML_PLUGIN__QML_ROSSIGNAL_HPP_

#include <QObject>
#include <QQuickItem>
#include <QVariant>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"

/**
 * @brief The RosSignal class provides a QML object that publishes on a
 * configurable topic an empty message (ie, a signal) every time signal() is
 * called.
 */
class RosSignal : public QObjectRos2
{
  Q_OBJECT
  Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:
  RosSignal() {}

  virtual ~RosSignal() {}

  void setTopic(QString topic);

  Q_INVOKABLE void signal();

  void onIncomingSignal(const std_msgs::msg::Empty);

signals:
  void triggered();

private:
  QString _topic;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _publisher;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscriber;
};

#endif  // ROS_QML_PLUGIN__QML_ROSSIGNAL_HPP_
