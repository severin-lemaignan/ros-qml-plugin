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


#include <QJsonDocument>

#include <rclcpp_action/rclcpp_action.hpp>
#include <communication_skills/action/chat.hpp>

#include "ros_qml_plugin/qml_chatskill.hpp"
#include "ros_qml_plugin/ros2.hpp"


void ChatSkill::start(QString prompt, QString initial_input)
{
  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  if (!_client) {
    // if not connected yet, do it now
    setAction("/skill/chat");
  }

  // if still not conencted, return
  if (!_client) {
    std::cerr << "Unable to connect to the Chat skill." << std::endl;
    return;
  }

  if (!_client->wait_for_action_server()) {
    std::cerr << "Chat skill server not available" << std::endl;
  }

  auto goal_msg = communication_skills::action::Chat::Goal();
  goal_msg.person_id = _person_id.toStdString();
  goal_msg.group_id = _group_id.toStdString();


  goal_msg.role.name = "__default__";
  goal_msg.role.configuration = QJsonDocument::fromVariant(
    QVariantMap(
  {
    {"prompt",
      prompt}})).toJson().toStdString();

  if (!initial_input.isEmpty()) {
    goal_msg.initiate = true;
    goal_msg.initial_input = initial_input.toStdString();
  }

  auto send_goal_options =
    rclcpp_action::Client<communication_skills::action::Chat>::SendGoalOptions();

  send_goal_options.goal_response_callback = std::bind(
    &ChatSkill::goal_response_callback, this, std::placeholders::_1);

  send_goal_options.feedback_callback = std::bind(
    &ChatSkill::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback = std::bind(
    &ChatSkill::result_callback, this, std::placeholders::_1);

  _goal_handle_future = _client->async_send_goal(goal_msg, send_goal_options);
}

void ChatSkill::stop()
{
  if (!_client) {
    std::cerr << "Chat skill client not initialized." << std::endl;
    return;
  }

  if (!_goal_handle_future.valid()) {
    std::cerr << "No active chat to stop." << std::endl;
    return;
  }

  auto cancel_future = _client->async_cancel_goal(_goal_handle_future.get());
  if (cancel_future.wait_for(std::chrono::seconds(1)) != std::future_status::timeout) {
    auto result = cancel_future.get();
    if (result->return_code == action_msgs::srv::CancelGoal::Response::ERROR_NONE) {
      std::cout << "Chat canceled successfully." << std::endl;
    } else {
      std::cerr << "Failed to cancel chat." << std::endl;
    }

  } else {
    std::cerr << "Chat cancel request timed out." << std::endl;
  }
}

void ChatSkill::goal_response_callback(
  rclcpp_action::ClientGoalHandle<communication_skills::action::Chat>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    std::cerr <<
      "Chat goal was rejected by server! Check the communication_hub logs to know more." <<
      std::endl;
    emit goalRejected();
    return;
  }
}

void ChatSkill::feedback_callback(
  rclcpp_action::ClientGoalHandle<communication_skills::action::Chat>::SharedPtr,
  const std::shared_ptr<const communication_skills::action::Chat::Feedback>)
{
  // TODO(SLE): expose feedback data
  emit feedbackReceived();
}

void ChatSkill::result_callback(
  const rclcpp_action::ClientGoalHandle<communication_skills::action::Chat>::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      std::cerr << "Chat was aborted" << std::endl;
      return;
    case rclcpp_action::ResultCode::CANCELED:
      std::cerr << "Chat was canceled" << std::endl;
      return;
    default:
      std::cerr << "Unknown result code" << std::endl;
      return;
  }
  _error_msg = QString::fromStdString(result.result->result.error_msg);
  // TODO(SLE): expose error code
  emit resultReceived();
}

template class RosActionImpl<communication_skills::action::Chat>;
