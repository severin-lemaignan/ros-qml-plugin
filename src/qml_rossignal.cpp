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

#include "ros_qml_plugin/qml_rossignal.hpp"
#include "ros_qml_plugin/ros2.hpp"

using std::placeholders::_1;

void RosSignal::setTopic(QString topic)
{

  if (topic == _topic) {
    return;
  }

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  _subscriber = node->create_subscription<std_msgs::msg::Empty>(
    topic.toStdString(), 1,
    std::bind(&RosSignal::onIncomingSignal, this, _1));

  _publisher =
    node->create_publisher<std_msgs::msg::Empty>(topic.toStdString(), 1);

  _topic = topic;
}

void RosSignal::onIncomingSignal(const std_msgs::msg::Empty /* msg */)
{
  emit triggered();
}

void RosSignal::signal()
{
  if (std::string(_publisher->get_topic_name()).empty()) {
    std::cerr << "RosSignal.signal() called without any topic." << std::endl;
    return;
  }

  _publisher->publish(std_msgs::msg::Empty());
}
