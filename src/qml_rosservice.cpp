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

#include <QQmlEngine>
#include <QQmlContext>
#include <chrono>
#include <i18n_msgs/srv/get_locales.hpp>

#include "ros_qml_plugin/qml_rosservice.hpp"
#include "ros_qml_plugin/ros2.hpp"

using namespace std::chrono_literals;

template<typename T> void RosServiceImpl<T>::setService(const QString & service)
{
  if (service == _service_name) {
    return;
  }

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  _service = node->create_service<T>(
    service.toStdString(), std::bind(
      &RosServiceImpl<T>::handle_request, this,
      std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);
}


///////////////////////////////////////////////////////////////////////////////

template<typename T> void RosServiceClientImpl<T>::setService(const QString & service)
{
  if (service == _service_name) {
    return;
  }

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  _client = node->create_client<T>(
    service.toStdString(), rmw_qos_profile_services_default);
}

///////////////////////////////////////////////////////////////////////////////

void SetBoolService::handle_request(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  emit requestReceived();
  bool value = request->data;
  if (value != _value) {
    _value = value;
    emit valueChanged();
  }
  response->success = true;
}

template class RosServiceImpl<std_srvs::srv::SetBool>;

///////////////////////////////////////////////////////////////////////////////

void GetLocalesService::callService()
{
  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  if (!_client) {
    std::cerr << "Service called without a client." << std::endl;
    return;
  }

  if (!_client->service_is_ready()) {
    std::cerr << "Service not available" << std::endl;
    return;
  }

  if (!rclcpp::ok()) {
    std::cerr << "ROS2 is not ok" << std::endl;
    return;
  }

  auto request = std::make_shared<i18n_msgs::srv::GetLocales::Request>();
  auto result_future =
    _client->async_send_request(
    request,
    std::bind(&GetLocalesService::handle_response, this, std::placeholders::_1));
}

void GetLocalesService::handle_response(
  rclcpp::Client<i18n_msgs::srv::GetLocales>::SharedFuture future)
{
  QStringList locales;
  auto result_locales = future.get()->locales;
  for (const auto & locale : result_locales) {
    locales.append(QString::fromStdString(locale));
  }
  if (locales != _locales) {
    _locales = locales;
  }
  emit resultReceived();
}

template class RosServiceClientImpl<i18n_msgs::srv::GetLocales>;

///////////////////////////////////////////////////////////////////////////////


void SetUiFragmentService::handle_request(
  const std::shared_ptr<ui_msgs::srv::SetUiFragment::Request> request,
  std::shared_ptr<ui_msgs::srv::SetUiFragment::Response> response)
{
  _qml_import_path = QString::fromStdString(request->qml_import_path);
  _qml_fragment = QString::fromStdString(request->qml_fragment);

  // inject the new import path
  if (!_qml_import_path.isEmpty()) {
    if (!QQmlEngine::contextForObject(this)) {
      std::cerr << "No QQmlEngine context for object" << std::endl;
      return;
    }

    std::cout << "Adding qml import path:" << _qml_import_path.toStdString() << std::endl;
    QQmlEngine::contextForObject(this)->engine()->addImportPath(
      _qml_import_path);
  }

  response->error_msg = "success";

  emit requestReceived();
}

template class RosServiceImpl<ui_msgs::srv::SetUiFragment>;
