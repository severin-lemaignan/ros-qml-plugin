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

#ifndef ROS_QML_PLUGIN__QML_CHATSKILL_HPP_
#define ROS_QML_PLUGIN__QML_CHATSKILL_HPP_

#include <QObject>
#include <QQuickItem>
#include <memory>

#include <communication_skills/action/chat.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"
#include "ros_qml_plugin/qml_rosaction.hpp"

class ChatSkill : public RosActionImpl<communication_skills::action::Chat>
{
  Q_OBJECT
  Q_PROPERTY(QString personId MEMBER _person_id)
  Q_PROPERTY(QString groupId MEMBER _group_id)
  Q_PROPERTY(QString errorMsg MEMBER _error_msg)

public:
  Q_INVOKABLE void start(QString prompt = "", QString initial_input = "");
  Q_INVOKABLE void stop();

private:
  QString _person_id;
  QString _group_id;
  QString _error_msg;

  std::shared_future<rclcpp_action::ClientGoalHandle<communication_skills::action::Chat>::SharedPtr>
  _goal_handle_future;

  void goal_response_callback(
    rclcpp_action::ClientGoalHandle<communication_skills::action::Chat>::SharedPtr);
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<communication_skills::action::Chat>::SharedPtr,
    const std::shared_ptr<const communication_skills::action::Chat::Feedback>);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<communication_skills::action::Chat>::WrappedResult &);
};


#endif  // ROS_QML_PLUGIN__QML_CHATSKILL_HPP_
