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


#include <rclcpp_action/rclcpp_action.hpp>
#include <navigation_skills/action/navigate_to_pose.hpp>

#include "ros_qml_plugin/qml_navigate_skill.hpp"
#include "ros_qml_plugin/ros2.hpp"
#include "ros_qml_plugin/ros_types.hpp"

void NavigateToPoseSkill::navigate_to_pose(const QVariant & pose, const QString & behavior_tree)
{
  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  if (!_client) {
    // if not connected yet, do it now
    setAction("/skill/navigate_to_pose");
  }

  // if still not connected, return
  if (!_client) {
    std::cerr << "Unable to connect to the NavigateToPose skill." << std::endl;
    return;
  }

  if (!_client->wait_for_action_server()) {
    std::cerr << "NavigateToPose skill server not available" << std::endl;
  }

  auto goal_msg = navigation_skills::action::NavigateToPose::Goal();

  if (pose.canConvert<RosPose>()) {
    _pose = pose;
    goal_msg.pose = _pose.value<RosPose>().toMsg();
  } else {
    qWarning() << "Invalid pose type passed to navigate_to_pose";
    return;
  }

  _behavior_tree = behavior_tree;
  goal_msg.behavior_tree = _behavior_tree.toStdString();

  auto send_goal_options =
    rclcpp_action::Client<navigation_skills::action::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback = std::bind(
    &NavigateToPoseSkill::goal_response_callback, this, std::placeholders::_1);

  send_goal_options.feedback_callback = std::bind(
    &NavigateToPoseSkill::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback = std::bind(
    &NavigateToPoseSkill::result_callback, this, std::placeholders::_1);

  auto goal_handle_future = _client->async_send_goal(goal_msg, send_goal_options);
}

void NavigateToPoseSkill::goal_response_callback(
  rclcpp_action::ClientGoalHandle<navigation_skills::action::NavigateToPose>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    std::cerr << "Goal was rejected by server" << std::endl;
    emit goalRejected();
    return;
  }
}

void NavigateToPoseSkill::feedback_callback(
  rclcpp_action::ClientGoalHandle<navigation_skills::action::NavigateToPose>::SharedPtr,
  const std::shared_ptr<const navigation_skills::action::NavigateToPose::Feedback>)
{
  // TODO(SLE): expose feedback data
  emit feedbackReceived();
}

void NavigateToPoseSkill::result_callback(
  const rclcpp_action::ClientGoalHandle<navigation_skills::action::NavigateToPose>::WrappedResult &
  result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      std::cerr << "Goal was aborted" << std::endl;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      std::cerr << "Goal was canceled" << std::endl;
      return;
    default:
      std::cerr << "Unknown result code" << std::endl;
      return;
  }
  _error_msg = QString::fromStdString(result.result->result.error_msg);
  // TODO(SLE): expose error code
  emit resultReceived();
}

template class RosActionImpl<navigation_skills::action::NavigateToPose>;
