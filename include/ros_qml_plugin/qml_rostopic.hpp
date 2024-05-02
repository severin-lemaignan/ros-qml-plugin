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

#ifndef ROS_QML_PLUGIN__QML_ROSTOPIC_HPP_
#define ROS_QML_PLUGIN__QML_ROSTOPIC_HPP_

#include <QObject>
#include <QQuickItem>
#include <QVariant>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"

/**
 * @brief A QtQuick item that publish/subscribe to a ROS2 topic of type
 * std_msgs/Int16.
 */
class RosTopic : public QObjectRos2 {
  Q_OBJECT
  Q_PROPERTY(QVariant value WRITE setValue MEMBER _value NOTIFY onValueChanged)
  Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:
  RosTopic() {}
  virtual ~RosTopic() {}

  virtual void setTopic(const QString &) = 0;
  virtual void setValue(const QVariant &) = 0;
  Q_INVOKABLE void publish() {}

signals:
  void onValueChanged();
  void messageReceived();

protected:
  QString _topic;
  QVariant _value;
};

template <typename T> class RosTopicImpl : public RosTopic {

public:
  RosTopicImpl<T>() {}
  virtual ~RosTopicImpl<T>() {}

  void setTopic(const QString &);
  void setValue(const QVariant &);
  Q_INVOKABLE void publish();

private:
  void onIncomingData(const T &data);

  typename rclcpp::Publisher<T>::SharedPtr _publisher;
  typename rclcpp::Subscription<T>::SharedPtr _subscriber;
};

#endif // ROS_QML_PLUGIN__QML_ROSTOPIC_HPP_
