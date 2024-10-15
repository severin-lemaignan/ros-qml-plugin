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

#include "ros_qml_plugin/qml_rosparam.hpp"
#include "ros_qml_plugin/ros2.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

RosParam::RosParam() {_node = Ros2Qml::getInstance().node();}

void RosParam::setValue(QVariant value)
{
  if (value == _value) {
    return;
  } else {
    _value = value;
  }

  if (!_is_ready) {
    std::cerr << "Parameter service not ready; have you called ready() in "
      "Component.onCompleted?"
              << std::endl;
    return;
  }

  bool remote_parameter = (_target_node_name != "");

  if (remote_parameter) {
    if (!_param_client || !_param_client->service_is_ready()) {
      std::cerr << "Cannot set parameter " << _name.toStdString() << " on node "
                << _target_node_name.toStdString()
                << ": parameter service not ready" << std::endl;
      return;
    }
  }

  std::shared_ptr<rclcpp::Parameter> parameter;
  switch (value.type()) {
    case QVariant::Bool:
      parameter = std::make_shared<rclcpp::Parameter>(
        _name.toStdString(),
        value.toBool());
      break;
    case QVariant::Int:
      parameter =
        std::make_shared<rclcpp::Parameter>(_name.toStdString(), value.toInt());
      break;
    case QVariant::Double:
      parameter = std::make_shared<rclcpp::Parameter>(
        _name.toStdString(),
        value.toDouble());
      break;
    case QVariant::String:
      parameter = std::make_shared<rclcpp::Parameter>(
        _name.toStdString(), value.toString().toStdString());
      break;
    case QVariant::UserType:
    case QVariant::StringList:
      {
        if (value.canConvert<QStringList>() && value.convert(QVariant::StringList)) 
        {
          auto stringList = value.toStringList();
          std::vector<std::string> v;
          v.reserve(stringList.size());  // Reserve memory for efficiency

          for (const QString &qStr : stringList) {
                v.push_back(qStr.toStdString());  // Convert each QString to std::string
          }

          parameter = std::make_shared<rclcpp::Parameter>(
                _name.toStdString(), v);

        }
        else {
            std::cerr << "Unsupported user type for parameter value: " <<  value.typeName()
                        << std::endl;
            }
      }
      break;
    default:
      std::cerr << "Unsupported type for parameter value: " <<  value.typeName()
                << std::endl;
  }

  if (parameter) {
    if (remote_parameter) {
      _param_client->set_parameters({*parameter});
    } else {
      _node->set_parameter(*parameter);
    }
  }
}

void RosParam::onRos2Initialized()
{
  if (_name.isEmpty()) {
    std::cerr << "Cannot configure a parameter without a name" << std::endl;
    return;
  }

  bool remote_parameter = (_target_node_name != "");

  if (remote_parameter) {
    ///////////////////////////////////////////////////////////////////////////
    // REMOTE PARAMETER
    //
    if (!Ros2Qml::getInstance().isInitialized()) {
      std::cerr
        << "ROS 2 not yet initialized! Cannot configure a remote parameter."
        << std::endl;
      return;
    }

    if (!_param_client) {
      std::cout << "Creating new parameter client for node "
                << _target_node_name.toStdString() << std::endl;
      _param_client = std::make_shared<rclcpp::AsyncParametersClient>(
        _node, _target_node_name.toStdString());
    }

    if (!_param_client->service_is_ready()) {
      bool ok = _param_client->wait_for_service(1s);
      if (!ok) {
        std::cerr << "Could not connect to parameter service for node "
                  << _target_node_name.toStdString() << std::endl;
        return;
      }
    }

    // add callback to update the value when the parameter changes
    _remote_cb = _param_client->on_parameter_event(
      std::bind(&RosParam::onRemoteParameterEvent, this, _1));
  } else {
    ///////////////////////////////////////////////////////////////////////////
    // LOCAL PARAMETER

    if (!_value.isValid()) {
        std::cerr << "Cannot configure a parameter without a type; set 'value' to "
        "a default value"
                << std::endl;
        return;
    }

    _local_cb = _node->add_on_set_parameters_callback(
      std::bind(&RosParam::onLocalParameterEvent, this, _1));

    QVariant updated_value = _value;

    if (_value.isValid()) {
      switch (_value.type()) {
        case QVariant::Bool:
          updated_value = QVariant::fromValue(
            _node->declare_parameter(_name.toStdString(), _value.toBool()));
          break;
        case QVariant::Int:
          updated_value = QVariant::fromValue(
            _node->declare_parameter(_name.toStdString(), _value.toInt()));
          break;
        case QVariant::Double:
          updated_value = QVariant::fromValue(
            _node->declare_parameter(_name.toStdString(), _value.toDouble()));
          break;
        case QVariant::String:
          updated_value =
            QVariant::fromValue(
            QString::fromStdString(
              _node->declare_parameter(
                _name.toStdString(), _value.toString().toStdString())));
          break;
        default:
          std::cerr << "Unsupported type for parameter value" << std::endl;
      }
    } else {
      std::cerr << "Need to specify an initial value for parameter "
                << _name.toStdString()
                << ", as ROS2 requires us to known the parameter type"
                << std::endl;
      // value = _node->declare_parameter(_name.toStdString());
    }

    // if the value was set at runtime, retrieve it and emit the signal
    if (updated_value.isValid() && updated_value != _value) {
      _value = updated_value;
      emit onValueChanged();
    }
  }
  _is_ready = true;
}

rcl_interfaces::msg::SetParametersResult RosParam::onLocalParameterEvent(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  QVariant value = _value;

  for (const auto & parameter : parameters) {
    if (parameter.get_name() == _name.toStdString()) {
      switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_BOOL:
          value = QVariant::fromValue(parameter.get_value<bool>());
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          value = QVariant::fromValue(parameter.get_value<int>());
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          value = QVariant::fromValue(parameter.get_value<double>());
          break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          value = QVariant::fromValue(
            QString::fromStdString(parameter.get_value<std::string>()));
          break;
        default:
          std::cerr << "Unsupported type " << parameter.get_type_name()
                    << " for ROS2 parameter " << parameter.get_name()
                    << std::endl;
      }

      if (value != _value) {
        _value = value;
        emit onValueChanged();
      }
    }
  }

  return result;
}

void RosParam::onRemoteParameterEvent(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  QVariant value = _value;

  for (const auto & p : event->changed_parameters) {
    if (p.name == _name.toStdString()) {
      switch (p.value.type) {
        case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
          value = QVariant::fromValue(p.value.bool_value);
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
          value = QVariant::fromValue(p.value.integer_value);
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
          value = QVariant::fromValue(p.value.double_value);
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
          value =
            QVariant::fromValue(QString::fromStdString(p.value.string_value));
          break;
        case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
          {
            QStringList stringList;
            for (const auto & s : p.value.string_array_value) {
              stringList.append(QString::fromStdString(s));
            }
            value = QVariant::fromValue(stringList);
          }
          break;
        default:
          std::cerr << "Unsupported type " << p.value.type
                    << " for ROS2 parameter " << p.name << " of node "
                    << _target_node_name.toStdString() << std::endl;
      }
      if (value != _value) {
        _value = value;
        emit onValueChanged();
      }

      break;
    }
  }
}
