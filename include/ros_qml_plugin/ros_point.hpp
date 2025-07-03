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

#ifndef ROS_QML_PLUGIN__ROS_POINT_HPP_
#define ROS_QML_PLUGIN__ROS_POINT_HPP_

#include <QObject>
#include <QString>
#include <geometry_msgs/msg/point_stamped.hpp>

class RosPoint
{
  Q_GADGET
  Q_PROPERTY(QString frame MEMBER _frame)
  Q_PROPERTY(double x MEMBER _x)
  Q_PROPERTY(double y MEMBER _y)
  Q_PROPERTY(double z MEMBER _z)

public:
  RosPoint() {}
  RosPoint(const QString & f, double x, double y, double z)
  : _frame(f), _x(x), _y(y), _z(z) {}

  QString _frame;
  double _x;
  double _y;
  double _z;

  geometry_msgs::msg::PointStamped toMsg() const
  {
    geometry_msgs::msg::PointStamped point;
    point.header.frame_id = _frame.toStdString();
    point.point.x = _x;
    point.point.y = _y;
    point.point.z = _z;
    return point;
  }
};

Q_DECLARE_METATYPE(RosPoint)

#endif  // ROS_QML_PLUGIN__ROS_POINT_HPP_
