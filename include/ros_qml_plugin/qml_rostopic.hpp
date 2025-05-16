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

#include <hri_actions_msgs/msg/closed_caption.hpp>
#include <hri_actions_msgs/msg/intent.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"

#define SHARED_CONSTANT(type, name, value) \
  Q_PROPERTY(type name READ name CONSTANT) \
  type name() const {return value;}


///////////////////////////////////////////////////////////////////////////////
/**
 * @brief A QtQuick item that publish/subscribe to a ROS2 topic of type
 * std_msgs/Int16.
 */
class RosTopic : public QObjectRos2
{
  Q_OBJECT
  Q_PROPERTY(QVariant value WRITE setValue MEMBER _value NOTIFY onValueChanged)
  Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
  Q_PROPERTY(bool isPublisher WRITE setIsPublisher MEMBER _is_publisher)
  Q_PROPERTY(bool isSubscriber WRITE setIsSubscriber MEMBER _is_subscriber)

public:
  RosTopic() {}
  virtual ~RosTopic() {}

  virtual void setTopic(const QString &) = 0;
  virtual void setValue(const QVariant &) = 0;
  virtual void setIsPublisher(const bool &) = 0;
  virtual void setIsSubscriber(const bool &) = 0;
  Q_INVOKABLE void publish() {}

signals:
  void onValueChanged();
  void messageReceived();

protected:
  QString _topic;
  QVariant _value;

  // by default, a RosTopic is both a publisher and a subscriber
  bool _is_publisher = true;
  bool _is_subscriber = true;
};

///////////////////////////////////////////////////////////////////////////////
template<typename T>
class RosTopicImpl : public RosTopic
{
public:
  RosTopicImpl<T>() {}
  virtual ~RosTopicImpl<T>() {}

  void setTopic(const QString &);
  void setValue(const QVariant &);
  void setIsPublisher(const bool &);
  void setIsSubscriber(const bool &);
  Q_INVOKABLE void publish();

protected:
  virtual void onIncomingData(const T & data);

  typename rclcpp::Publisher<T>::SharedPtr _publisher;
  typename rclcpp::Subscription<T>::SharedPtr _subscriber;
};

///////////////////////////////////////////////////////////////////////////////
class ClosedCaptionTopic
  : public RosTopicImpl<hri_actions_msgs::msg::ClosedCaption>
{
  Q_OBJECT
  Q_PROPERTY(QString speaker_id MEMBER _speaker_id)

protected:
  void
  onIncomingData(const hri_actions_msgs::msg::ClosedCaption & data) override;

private:
  QString _speaker_id;
};

///////////////////////////////////////////////////////////////////////////////
class IntentTopic
  : public RosTopicImpl<hri_actions_msgs::msg::Intent>
{
  Q_OBJECT
  Q_PROPERTY(QString data MEMBER _data)

  // intents
  SHARED_CONSTANT(
    QString, WakeUp,
    QString::fromStdString(hri_actions_msgs::msg::Intent::WAKEUP))
  SHARED_CONSTANT(
    QString, Suspend,
    QString::fromStdString(hri_actions_msgs::msg::Intent::SUSPEND))
  SHARED_CONSTANT(
    QString, RawUserInput,
    QString::fromStdString(hri_actions_msgs::msg::Intent::RAW_USER_INPUT))
  SHARED_CONSTANT(
    QString, EngageWith,
    QString::fromStdString(hri_actions_msgs::msg::Intent::ENGAGE_WITH))
  SHARED_CONSTANT(
    QString, Guide,
    QString::fromStdString(hri_actions_msgs::msg::Intent::GUIDE))
  SHARED_CONSTANT(
    QString, GrabObject,
    QString::fromStdString(hri_actions_msgs::msg::Intent::GRAB_OBJECT))
  SHARED_CONSTANT(
    QString, BringObject,
    QString::fromStdString(hri_actions_msgs::msg::Intent::BRING_OBJECT))
  SHARED_CONSTANT(
    QString, PlaceObject,
    QString::fromStdString(hri_actions_msgs::msg::Intent::PLACE_OBJECT))
  SHARED_CONSTANT(
    QString, Greet,
    QString::fromStdString(hri_actions_msgs::msg::Intent::GREET))
  SHARED_CONSTANT(
    QString, Say,
    QString::fromStdString(hri_actions_msgs::msg::Intent::SAY))
  SHARED_CONSTANT(
    QString, PresentContent,
    QString::fromStdString(hri_actions_msgs::msg::Intent::PRESENT_CONTENT))
  SHARED_CONSTANT(
    QString, PerformMotion,
    QString::fromStdString(hri_actions_msgs::msg::Intent::PERFORM_MOTION))
  SHARED_CONSTANT(
    QString, StartActivity,
    QString::fromStdString(hri_actions_msgs::msg::Intent::START_ACTIVITY))
  SHARED_CONSTANT(
    QString, StopActivity,
    QString::fromStdString(hri_actions_msgs::msg::Intent::STOP_ACTIVITY))

  // modalities
  Q_PROPERTY(QString modality MEMBER _modality)
  SHARED_CONSTANT(
    QString, ModalityTouchscreen,
    QString::fromStdString(hri_actions_msgs::msg::Intent::MODALITY_TOUCHSCREEN))
  SHARED_CONSTANT(
    QString, ModalitySpeech,
    QString::fromStdString(hri_actions_msgs::msg::Intent::MODALITY_SPEECH))
  SHARED_CONSTANT(
    QString, ModalityMotion,
    QString::fromStdString(hri_actions_msgs::msg::Intent::MODALITY_MOTION))
  SHARED_CONSTANT(
    QString, ModalityOther,
    QString::fromStdString(hri_actions_msgs::msg::Intent::MODALITY_OTHER))
  SHARED_CONSTANT(
    QString, ModalityInternal,
    QString::fromStdString(hri_actions_msgs::msg::Intent::MODALITY_INTERNAL))

  // source
  Q_PROPERTY(QString source MEMBER _source)
  SHARED_CONSTANT(
    QString, SourceRobotItself,
    QString::fromStdString(hri_actions_msgs::msg::Intent::ROBOT_ITSELF))
  SHARED_CONSTANT(
    QString, SourceRemoteSupervisor,
    QString::fromStdString(hri_actions_msgs::msg::Intent::REMOTE_SUPERVISOR))
  SHARED_CONSTANT(
    QString, SourceUnknownAgent,
    QString::fromStdString(hri_actions_msgs::msg::Intent::UNKNOWN_AGENT))
  SHARED_CONSTANT(
    QString, SourceUnknown,
    QString::fromStdString(hri_actions_msgs::msg::Intent::UNKNOWN))

public:
  Q_INVOKABLE void publish();

protected:
  void
  onIncomingData(const hri_actions_msgs::msg::Intent & data) override;

private:
  QString _data;
  QString _modality;
  QString _source;
};


#endif  // ROS_QML_PLUGIN__QML_ROSTOPIC_HPP_
