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

#pragma once

#include <QObject>
#include <QVariant>
#include "ros_qml_plugin/ros_types.hpp"

/** Singleton class to provide a QML interface for ROS functionality.
 * This class allows creating ROS points and can be extended to include more
 * ROS-related functionalities.
 */
class Ros : public QObject
{
  Q_OBJECT

public:
  explicit Ros(QObject * parent = nullptr)
  : QObject(parent) {}

  Q_INVOKABLE QVariant point(const QString & frame, double x, double y, double z)
  {
    return QVariant::fromValue(RosPoint(frame, x, y, z));
  }

  Q_INVOKABLE QVariant point(double x, double y, double z)
  {
    return QVariant::fromValue(RosPoint(x, y, z));
  }

  Q_INVOKABLE QVariant pose(const QString & frame, double x, double y, double z)
  {
    return QVariant::fromValue(RosPose(frame, x, y, z));
  }

  Q_INVOKABLE QVariant pose(double x, double y, double z)
  {
    return QVariant::fromValue(RosPose(x, y, z));
  }

  Q_INVOKABLE QVariant pose(
    const QString & frame, double x, double y, double z, double qx,
    double qy, double qz, double qw)
  {
    return QVariant::fromValue(RosPose(frame, x, y, z, qx, qy, qz, qw));
  }

  Q_INVOKABLE QVariant pose(
    double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    return QVariant::fromValue(RosPose(x, y, z, qx, qy, qz, qw));
  }

};
