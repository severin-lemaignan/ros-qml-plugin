// Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
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

#ifndef ROS_QML_PLUGIN__QML_SET_EXPRESSION_SKILL_HPP_
#define ROS_QML_PLUGIN__QML_SET_EXPRESSION_SKILL_HPP_

#include <QObject>
#include <QQuickItem>
#include <memory>

#include <interaction_skills/msg/set_expression.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"

class SetExpressionSkill : public QObjectRos2
{
  Q_OBJECT

public:
  SetExpressionSkill();
  Q_INVOKABLE void set_expression(QString expression);
  Q_INVOKABLE void set_expression(float valence, float arousal);

private:
  typename rclcpp::Publisher<interaction_skills::msg::SetExpression>::SharedPtr _publisher;
};

#endif  // ROS_QML_PLUGIN__QML_SET_EXPRESSION_SKILL_HPP_
