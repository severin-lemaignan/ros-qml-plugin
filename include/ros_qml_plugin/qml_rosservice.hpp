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

#include <atomic>
#include <functional>
#include <iostream>

#include <i18n_msgs/srv/get_locales.hpp>
#include <QString>
#include <QVector>
#include <rclcpp/rclcpp.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"
#include "ros_qml_plugin/ros2.hpp"


template<typename ServiceT>
class RosService
{
public:
  using RequestPtr = typename ServiceT::Request::SharedPtr;
  using Client = typename rclcpp::Client<ServiceT>;
  using ClientPtr = typename Client::SharedPtr;
  using SharedFutureResponse = typename Client::SharedFuture;
  using SharedFutureAndRequestId = typename Client::SharedFutureAndRequestId;
  using CallbackType = typename std::function<void (SharedFutureResponse)>;

  RosService(const std::string & topic)
  : topic_(topic)
  {
    node_ = Ros2Qml::getInstance().node();
    cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = node->create_client<ServiceT>(topic, rmw_qos_profile_services_default, cb_group_);
  }

  void cancel()
  {
    client->prune_pending_requests();
    ready_ = true;
  }

protected:
  ~RosService()
  {
    cancel();
  }

  bool request(RequestPtr request, CallbackType callback, rclcpp::Duration timeout)
  {
    using namespace std::placeholders;

    if (!client_->service_is_ready()) {
      std::cerr << "Service server " << topic_ << " not ready";
      return false;
    }

    if (!ready_.exchange(false)) {
      std::cerr << "Service client  " << topic_ << " not ready";
      ready_ = true;
      return false;
    }

    auto future_and_id = client_->async_send_request(
      request, std::bind(&onResponse, this, callback, _1));
    rclcpp::create_timer(
      node_, node->get_clock(), timeout, std::bind(&onTimeout, this, callback, future_and_id));

    return true;
  }

private:
  void onResponse(CallbackType callback, SharedFutureResponse future)
  {
    timeout_timer_.reset();
    ready_ = true;
    callback(future);
  }

  void onTimeout(CallbackType callback, SharedFutureAndRequestId future_and_id)
  {
    client_->remove_pending_request(future_and_id);
    timeout_timer_.reset();
    callback(SharedFutureResponse());
  }

  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  ClientPtr client_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  std::atomic_bool ready_;
};

///////////////////////////////////////////////////////////////////////////////
class GetLocalesService
: public RosService<i18n_msgs::srv::GetLocales>, QObjectRos2
{
  Q_OBJECT

public:
  GetLocalesService();
  ~GetLocalesService();
  bool request();

signals:
  void onResponse(bool success, QVector<QString> locales);
};


#endif  // ROS_QML_PLUGIN__QML_ROSSERVICE_HPP_
