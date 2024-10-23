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

#include <i18n_msgs/action/set_locale.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include "ros_qml_plugin/qml_rosaction.hpp"
#include "ros_qml_plugin/ros2.hpp"

#include <chrono>

using namespace std::chrono_literals;


template<typename T> void RosActionImpl<T>::setAction(const QString & action)
{
  if (action == _action) {
    return;
  }

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  _client = rclcpp_action::create_client<T>(
    node, action.toStdString(), _cb_group_2);

  std::cout << "Action set" << std::endl;

}

void SetLocaleAction::sendGoal()
{
  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  if (!_client) {
    std::cerr << "Action called without a client." << std::endl;
    return;
  }

  if (!_client->wait_for_action_server()) {
    std::cerr << "Action server not available" << std::endl;
  }

  if (!rclcpp::ok()) {
    std::cerr << "ROS2 is not ok" << std::endl;
  }

  auto goal_msg = i18n_msgs::action::SetLocale::Goal();
  goal_msg.locale = _locale.toStdString();

  auto send_goal_options = rclcpp_action::Client<i18n_msgs::action::SetLocale>::SendGoalOptions();
  send_goal_options.feedback_callback = std::bind(&SetLocaleAction::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

  auto goal_handle_future = _client->async_send_goal(goal_msg, send_goal_options);
  goal_handle_future.wait_for(3s);
  rclcpp_action::ClientGoalHandle<i18n_msgs::action::SetLocale>::SharedPtr goal_handle =
    goal_handle_future.get();
  if (!goal_handle) {
    std::cerr << "Goal was rejected by server" << std::endl;
    return;
  }

  auto result_future = _client->async_get_result(goal_handle);
  result_future.wait_for(3s);
  rclcpp_action::ClientGoalHandle<i18n_msgs::action::SetLocale>::WrappedResult wrapped_result =
    result_future.get();
  switch (wrapped_result.code) {
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

  _error_msg = QString::fromStdString(wrapped_result.result->error_msg);
  emit resultReceived();

  // node.reset();
}

void SetLocaleAction::feedback_callback(
  rclcpp_action::ClientGoalHandle<i18n_msgs::action::SetLocale>::SharedPtr,
  const std::shared_ptr<const i18n_msgs::action::SetLocale::Feedback> feedback)
{
  _progress = QString::fromStdString(feedback->progress);
  emit feedbackReceived();
}

template class RosActionImpl<i18n_msgs::action::SetLocale>;
