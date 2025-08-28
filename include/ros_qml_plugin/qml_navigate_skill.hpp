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

#ifndef ROS_QML_PLUGIN__QML_NAVIGATE_SKILL_HPP_
#define ROS_QML_PLUGIN__QML_NAVIGATE_SKILL_HPP_

#include <QObject>
#include <QQuickItem>
#include <memory>

#include <navigation_skills/action/navigate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"
#include "ros_qml_plugin/qml_rosaction.hpp"

class NavigateSkill : public RosActionImpl<navigation_skills::action::Navigate>
{
  Q_OBJECT
  Q_PROPERTY(QVariant pose MEMBER _pose)
  Q_PROPERTY(QString target MEMBER _target)
  Q_PROPERTY(QString errorMsg MEMBER _error_msg)

public:
  Q_INVOKABLE void navigate(const QVariant & target);

private:
  QVariant _pose;
  QString _target;
  QString _error_msg;

  void goal_response_callback(
    rclcpp_action::ClientGoalHandle<navigation_skills::action::Navigate>::SharedPtr);
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<navigation_skills::action::Navigate>::SharedPtr,
    const std::shared_ptr<const navigation_skills::action::Navigate::Feedback>);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<navigation_skills::action::Navigate>::
    WrappedResult &);
};


#endif  // ROS_QML_PLUGIN__QML_NAVIGATE_SKILL_HPP_
