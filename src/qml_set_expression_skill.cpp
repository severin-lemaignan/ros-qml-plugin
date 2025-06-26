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


#include "ros_qml_plugin/qml_set_expression_skill.hpp"
#include "ros_qml_plugin/ros2.hpp"


SetExpressionSkill::SetExpressionSkill()
{
  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();
  _publisher = node->create_publisher<interaction_skills::msg::SetExpression>(
    "/skill/set_expression", 1);
}

void SetExpressionSkill::set_expression(QString expression)
{
  interaction_skills::msg::SetExpression message;

  message.expression.expression = expression.toStdString();

  _publisher->publish(message);

}

void SetExpressionSkill::set_expression(float valence, float arousal)
{
  interaction_skills::msg::SetExpression message;

  message.expression.valence = valence;
  message.expression.arousal = arousal;

  _publisher->publish(message);
}
