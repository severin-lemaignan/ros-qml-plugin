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

#ifndef ROS_QML_PLUGIN__QML_ROSPARAM_HPP_
#define ROS_QML_PLUGIN__QML_ROSPARAM_HPP_

#include <QObject>
#include <QQuickItem>
#include <QVariant>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"

typedef rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr
    ParameterEventSubscription;

class RosParam : public QObjectRos2 {
  Q_OBJECT
  Q_PROPERTY(QString node MEMBER _target_node_name)
  Q_PROPERTY(QString name MEMBER _name)
  Q_PROPERTY(QVariant value WRITE setValue MEMBER _value NOTIFY onValueChanged)

public:
  RosParam();

  virtual ~RosParam() {}

  void setValue(QVariant value);

  /**
   * Configure the parameter service + callback
   */
  Q_INVOKABLE void ready();

signals:
  void onValueChanged();

private:
  // for local parameters
  rcl_interfaces::msg::SetParametersResult
  onLocalParameterEvent(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _local_cb;

  // for remote parameters
  rclcpp::SyncParametersClient::SharedPtr _param_client;

  void onRemoteParameterEvent(
      const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  ParameterEventSubscription _remote_cb;

  //////////////////
  bool _is_ready = false;
  QString _target_node_name;
  QString _name;
  QVariant _value;
  rclcpp::Node::SharedPtr _node;
};

#endif // ROS_QML_PLUGIN__QML_ROSPARAM_HPP_
