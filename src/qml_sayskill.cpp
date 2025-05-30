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
#include <communication_skills/action/say.hpp>

#include "ros_qml_plugin/qml_sayskill.hpp"
#include "ros_qml_plugin/ros2.hpp"


void SaySkill::say(QString input)
{


  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  if (!_client) {
    // if not connected yet, do it now
    setAction("/say");
  }

  // if still not conencted, return
  if (!_client) {
    std::cerr << "Unable to connect to the Say skill." << std::endl;
    return;
  }

  if (!_client->wait_for_action_server()) {
    std::cerr << "Say skill server not available" << std::endl;
  }

  auto goal_msg = communication_skills::action::Say::Goal();
  goal_msg.person_id = _person_id.toStdString();
  goal_msg.group_id = _group_id.toStdString();
  goal_msg.input = input.toStdString();

  auto send_goal_options =
    rclcpp_action::Client<communication_skills::action::Say>::SendGoalOptions();

  send_goal_options.goal_response_callback = std::bind(
    &SaySkill::goal_response_callback, this, std::placeholders::_1);

  send_goal_options.feedback_callback = std::bind(
    &SaySkill::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  send_goal_options.result_callback = std::bind(
    &SaySkill::result_callback, this, std::placeholders::_1);

  auto goal_handle_future = _client->async_send_goal(goal_msg, send_goal_options);
}

void SaySkill::goal_response_callback(
  rclcpp_action::ClientGoalHandle<communication_skills::action::Say>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    std::cerr << "Goal was rejected by server" << std::endl;
    emit goalRejected();
    return;
  }
}

void SaySkill::feedback_callback(
  rclcpp_action::ClientGoalHandle<communication_skills::action::Say>::SharedPtr,
  const std::shared_ptr<const communication_skills::action::Say::Feedback>)
{
  // TODO: expose feedback data
  emit feedbackReceived();
}

void SaySkill::result_callback(
  const rclcpp_action::ClientGoalHandle<communication_skills::action::Say>::WrappedResult & result)
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
  // TODO: expose error code
  emit resultReceived();
}

template class RosActionImpl<communication_skills::action::Say>;
