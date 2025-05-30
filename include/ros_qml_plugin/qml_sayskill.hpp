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

#ifndef ROS_QML_PLUGIN__QML_SAYSKILL_HPP_
#define ROS_QML_PLUGIN__QML_SAYSKILL_HPP_

#include <QObject>
#include <QQuickItem>
#include <memory>

#include <communication_skills/action/say.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"
#include "ros_qml_plugin/qml_rosaction.hpp"

class SaySkill : public RosActionImpl<communication_skills::action::Say>
{
  Q_OBJECT
  Q_PROPERTY(QString personId MEMBER _person_id)
  Q_PROPERTY(QString groupId MEMBER _group_id)
  Q_PROPERTY(QString errorMsg MEMBER _error_msg)

public:
  Q_INVOKABLE void say(QString input);

private:
  QString _person_id;
  QString _group_id;
  QString _error_msg;

  void goal_response_callback(
    rclcpp_action::ClientGoalHandle<communication_skills::action::Say>::SharedPtr);
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<communication_skills::action::Say>::SharedPtr,
    const std::shared_ptr<const communication_skills::action::Say::Feedback>);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<communication_skills::action::Say>::WrappedResult &);
};


#endif  // ROS_QML_PLUGIN__QML_SAYSKILL_HPP_
