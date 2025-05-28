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

#ifndef ROS_QML_PLUGIN__QML_ROSACTION_HPP_
#define ROS_QML_PLUGIN__QML_ROSACTION_HPP_

#include <QObject>
#include <QQuickItem>
#include <memory>

#include <i18n_msgs/action/set_locale.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"
#include "ros_qml_plugin/ros2.hpp"

class RosAction : public QObjectRos2
{
  Q_OBJECT
  Q_PROPERTY(QString action WRITE setAction MEMBER _action)

public:
  RosAction() {}
  virtual ~RosAction() {}

  virtual void setAction(const QString &) = 0;

signals:
  void goalRejected();
  void feedbackReceived();
  void resultReceived();

protected:
  QString _action;
};

///////////////////////////////////////////////////////////////////////////////

template<typename T>
class RosActionImpl : public RosAction
{
public:
  RosActionImpl<T>() {}
  virtual ~RosActionImpl<T>() {}

  void setAction(const QString & action)
  {
    if (action == _action) {
      return;
    }

    std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

    _client = rclcpp_action::create_client<T>(
      node, action.toStdString());

    std::cout << "Action set" << std::endl;
  }

protected:
  typename rclcpp_action::Client<T>::SharedPtr _client;
  rclcpp::CallbackGroup::SharedPtr _cb_group;
};

///////////////////////////////////////////////////////////////////////////////

class SetLocaleAction : public RosActionImpl<i18n_msgs::action::SetLocale>
{
  Q_OBJECT
  Q_PROPERTY(QString locale MEMBER _locale)
  Q_PROPERTY(QString errorMsg MEMBER _error_msg)
  Q_PROPERTY(QString progress MEMBER _progress)

public:
  Q_INVOKABLE void sendGoal();

private:
  QString _locale;
  QString _error_msg;
  QString _progress;
  void goal_response_callback(
    rclcpp_action::ClientGoalHandle<i18n_msgs::action::SetLocale>::SharedPtr);
  void feedback_callback(
    rclcpp_action::ClientGoalHandle<i18n_msgs::action::SetLocale>::SharedPtr,
    const std::shared_ptr<const i18n_msgs::action::SetLocale::Feedback>);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<i18n_msgs::action::SetLocale>::WrappedResult &);
};


#endif  // ROS_QML_PLUGIN__QML_ROSACTION_HPP_
