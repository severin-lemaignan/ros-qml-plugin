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

#ifndef ROS_QML_PLUGIN__QML_ROSSERVICE_HPP_
#define ROS_QML_PLUGIN__QML_ROSSERVICE_HPP_

#include <QObject>
#include <QQuickItem>

#include <memory>

#include <i18n_msgs/srv/get_locales.hpp>
#include <ui_msgs/srv/set_ui_fragment.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"

class RosService : public QObjectRos2
{
  Q_OBJECT
  Q_PROPERTY(QString service WRITE setService MEMBER _service_name)

public:
  RosService() {}
  virtual ~RosService() {}

  virtual void setService(const QString &) = 0;

signals:
  void requestReceived();

protected:
  QString _service_name;
};

template<typename T>
class RosServiceImpl : public RosService
{
public:
  RosServiceImpl<T>() {}
  virtual ~RosServiceImpl<T>() {}

  void setService(const QString &);

protected:
  virtual void handle_request(
    const std::shared_ptr<typename T::Request>,
    std::shared_ptr<typename T::Response>) {}

  typename rclcpp::Service<T>::SharedPtr _service;
  // rclcpp::CallbackGroup::SharedPtr _cb_group;
};

///////////////////////////////////////////////////////////////////////////////

class RosServiceClient : public QObjectRos2
{
  Q_OBJECT
  Q_PROPERTY(QString service WRITE setService MEMBER _service_name)

public:
  RosServiceClient() {}
  virtual ~RosServiceClient() {}

  virtual void setService(const QString &) = 0;

signals:
  void resultReceived();

protected:
  QString _service_name;
};

template<typename T>
class RosServiceClientImpl : public RosServiceClient
{
public:
  RosServiceClientImpl<T>() {}
  virtual ~RosServiceClientImpl<T>() {}

  void setService(const QString &);

protected:
  typename rclcpp::Client<T>::SharedPtr _client;
  // rclcpp::CallbackGroup::SharedPtr _cb_group;
};


///////////////////////////////////////////////////////////////////////////////

class GetLocalesService : public RosServiceClientImpl<i18n_msgs::srv::GetLocales>
{
  Q_OBJECT
  Q_PROPERTY(QStringList locales MEMBER _locales)

public:
  Q_INVOKABLE void callService();

private:
  QStringList _locales;
  void handle_response(rclcpp::Client<i18n_msgs::srv::GetLocales>::SharedFuture future);
};

///////////////////////////////////////////////////////////////////////////////

class SetUiFragmentService : public RosServiceImpl<ui_msgs::srv::SetUiFragment>
{
  Q_OBJECT
  Q_PROPERTY(QString qml_import_path MEMBER _qml_import_path)
  Q_PROPERTY(QString qml_fragment MEMBER _qml_fragment)

private:
  QString _qml_import_path;
  QString _qml_fragment;

  void handle_request(
    const std::shared_ptr<ui_msgs::srv::SetUiFragment::Request> request,
    std::shared_ptr<ui_msgs::srv::SetUiFragment::Response> response) override;
};


#endif  // ROS_QML_PLUGIN__QML_ROSSERVICE_HPP_
