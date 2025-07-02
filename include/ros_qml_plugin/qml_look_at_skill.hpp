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

#ifndef ROS_QML_PLUGIN__QML_LOOK_AT_SKILL_HPP_
#define ROS_QML_PLUGIN__QML_LOOK_AT_SKILL_HPP_

#include <QObject>
#include <QQuickItem>
#include <memory>

#include <interaction_skills/action/look_at.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"
#include "ros_qml_plugin/qml_rosaction.hpp"

#define SHARED_CONSTANT(type, name, value) \
  Q_PROPERTY(type name READ name CONSTANT) \
  type name() const {return value;}

class RosPoint : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QString frame MEMBER _frame)
  Q_PROPERTY(double x MEMBER _x)
  Q_PROPERTY(double y MEMBER _y)
  Q_PROPERTY(double z MEMBER _z)

public:
  QString _frame;
  double _x;
  double _y;
  double _z;

  geometry_msgs::msg::PointStamped toMsg() const
  {
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = _frame.toStdString();
    point.point.x = _x;
    point.point.y = _y;
    point.point.z = _z;
    return point;
  }
};


class LookAtSkill : public RosActionImpl<interaction_skills::action::LookAt>
{
  Q_OBJECT
  Q_PROPERTY(QString errorMsg MEMBER _error_msg)

  // policies
  SHARED_CONSTANT(
    QString, AUTO,
    QString::fromStdString(interaction_skills::action::LookAt::Goal::AUTO))
  SHARED_CONSTANT(
    QString, RESET,
    QString::fromStdString(interaction_skills::action::LookAt::Goal::RESET))
  SHARED_CONSTANT(
    QString, RANDOM,
    QString::fromStdString(interaction_skills::action::LookAt::Goal::RANDOM))
  SHARED_CONSTANT(
    QString, SOCIAL,
    QString::fromStdString(interaction_skills::action::LookAt::Goal::SOCIAL))
  SHARED_CONSTANT(
    QString, GLANCE,
    QString::fromStdString(interaction_skills::action::LookAt::Goal::GLANCE))

public:
  Q_INVOKABLE void look_at(const RosPoint * target, const QString & policy = "");
  Q_INVOKABLE void glance(const RosPoint * target);
  Q_INVOKABLE void look_at_faces();
  Q_INVOKABLE void look_around_randomly();

private:
  QString _error_msg;

  void goal_response_callback(
    rclcpp_action::ClientGoalHandle<interaction_skills::action::LookAt>::SharedPtr);
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<interaction_skills::action::LookAt>::SharedPtr,
    const std::shared_ptr<const interaction_skills::action::LookAt::Feedback>);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<interaction_skills::action::LookAt>::WrappedResult &);
};


#endif  // ROS_QML_PLUGIN__QML_LOOK_AT_SKILL_HPP_
