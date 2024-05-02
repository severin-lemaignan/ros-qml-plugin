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

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros_qml_plugin/qml_rostopic.hpp"
#include "ros_qml_plugin/ros2.hpp"

using std::placeholders::_1;

template <typename T> void RosTopicImpl<T>::onIncomingData(const T &data) {

  QVariant value;

  // special case std::string, as they are not directly convertible to QVariant
  if constexpr (std::is_same_v<T, std_msgs::msg::String>) {
    value = QVariant::fromValue(QString::fromStdString(data.data));
  } else {
    value = QVariant::fromValue(data.data);
  }

  if (value != _value) {
    _value = value;
    emit onValueChanged();
  }

  // always emit the signal to signal a message has been published, even if the
  // value did not change
  emit messageReceived();
}

template <typename T> void RosTopicImpl<T>::setTopic(const QString &topic) {
  if (topic == _topic) {
    return;
  }

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  _subscriber = node->create_subscription<T>(
      topic.toStdString(), 1,
      std::bind(&RosTopicImpl<T>::onIncomingData, this, _1));

  _publisher = node->create_publisher<T>(topic.toStdString(), 1);

  _topic = topic;
}

template <typename T> void RosTopicImpl<T>::setValue(const QVariant &value) {

  if (value == _value) {
    return;
  }

  _value = value;
  publish();
}

template <typename T> void RosTopicImpl<T>::publish() {

  if (!_publisher) {
    std::cerr << "RosTopic.publish() called without a publisher." << std::endl;
    return;
  }

  if (std::string(_publisher->get_topic_name()).empty()) {
    std::cerr << "RosTopic.publish() called without any topic." << std::endl;
    return;
  }

  T message;

  // special case std::string, as they are not directly convertible from
  // QVariant
  if constexpr (std::is_same_v<T, std_msgs::msg::String>) {
    message.data = _value.value<QString>().toStdString();
  } else {
    message.data = _value.value<decltype(T::data)>();
  }

  _publisher->publish(message);
}

template class RosTopicImpl<std_msgs::msg::Int16>;
template class RosTopicImpl<std_msgs::msg::Float32>;
template class RosTopicImpl<std_msgs::msg::Bool>;
template class RosTopicImpl<std_msgs::msg::String>;
