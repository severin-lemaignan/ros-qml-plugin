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

#include "ros_qml_plugin/qml_look_at_skill.hpp"
#include "ros_qml_plugin/ros2.hpp"
#include "ros_qml_plugin/ros_point.hpp"


void LookAtSkill::look_at(QVariant v_target, const QString & policy)
{
  using namespace std::placeholders;


  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  if (!_client) {
    // if not connected yet, do it now
    setAction("/skill/look_at");
  }

  // if still not connected, return
  if (!_client) {
    std::cerr << "Unable to connect to the LookAt skill." << std::endl;
    return;
  }

  if (!_client->wait_for_action_server()) {
    std::cerr << "LookAt skill server not available" << std::endl;
  }

  auto goal_msg = interaction_skills::action::LookAt::Goal();
  goal_msg.policy = policy.toStdString();

  // v_target invalid means that no target is set, which is fine (some
  // policies do not require a target)
  if (v_target.isValid()) {
    if (v_target.canConvert<RosPoint>()) {
      RosPoint target = v_target.value<RosPoint>();
      goal_msg.target = target.toMsg();
    } else {
      qWarning() << "Invalid point type passed look_at";
      return;
    }
  }

  auto send_goal_options =
    rclcpp_action::Client<interaction_skills::action::LookAt>::SendGoalOptions();

  send_goal_options.goal_response_callback = std::bind(
    &LookAtSkill::goal_response_callback, this, _1);

  send_goal_options.feedback_callback = std::bind(
    &LookAtSkill::feedback_callback, this, _1, _2);

  send_goal_options.result_callback = std::bind(
    &LookAtSkill::result_callback, this, _1);

  qInfo() << "Sending LookAt goal with policy " << policy << " and target "
          << v_target.toString();
  auto goal_handle_future = _client->async_send_goal(goal_msg, send_goal_options);
}

void LookAtSkill::glance(QVariant target)
{
  look_at(
    target,
    QString::fromStdString(interaction_skills::action::LookAt::Goal::GLANCE));
}

void LookAtSkill::look_at_faces()
{
  look_at(
    QVariant(),
    QString::fromStdString(interaction_skills::action::LookAt::Goal::SOCIAL));
}

void LookAtSkill::look_around_randomly()
{
  look_at(
    QVariant(),
    QString::fromStdString(interaction_skills::action::LookAt::Goal::RANDOM));
}

void LookAtSkill::reset()
{
  look_at(
    QVariant(),
    QString::fromStdString(interaction_skills::action::LookAt::Goal::RESET));
}


void LookAtSkill::goal_response_callback(
  rclcpp_action::ClientGoalHandle<interaction_skills::action::LookAt>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    std::cerr << "Goal was rejected by server" << std::endl;
    emit goalRejected();
    return;
  }
}

void LookAtSkill::feedback_callback(
  rclcpp_action::ClientGoalHandle<interaction_skills::action::LookAt>::SharedPtr,
  const std::shared_ptr<const interaction_skills::action::LookAt::Feedback>)
{
  // TODO(SLE): expose feedback data
  emit feedbackReceived();
}

void LookAtSkill::result_callback(
  const rclcpp_action::ClientGoalHandle<interaction_skills::action::LookAt>::WrappedResult &
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

template class RosActionImpl<interaction_skills::action::LookAt>;
