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

#ifndef ROS_QML_PLUGIN__ROS_TYPES_HPP_
#define ROS_QML_PLUGIN__ROS_TYPES_HPP_

#include <QObject>
#include <QString>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

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

class RosPose
{
  Q_GADGET
  Q_PROPERTY(QString frame MEMBER _frame)
  Q_PROPERTY(double x MEMBER _x)
  Q_PROPERTY(double y MEMBER _y)
  Q_PROPERTY(double z MEMBER _z)
  Q_PROPERTY(double qx MEMBER _qx)
  Q_PROPERTY(double qy MEMBER _qy)
  Q_PROPERTY(double qz MEMBER _qz)
  Q_PROPERTY(double qw MEMBER _qw)

public:
  RosPose() {}
  RosPose(const QString & f, double x, double y, double z)
  : _frame(f), _x(x), _y(y), _z(z), _qx(0.0), _qy(0.0), _qz(0.0), _qw(1.0) {}
  RosPose(
    const QString & f, double x, double y, double z,
    double qx, double qy, double qz, double qw)
  : _frame(f), _x(x), _y(y), _z(z), _qx(qx), _qy(qy), _qz(qz), _qw(qw) {}

  QString _frame;
  double _x;
  double _y;
  double _z;
  double _qx;
  double _qy;
  double _qz;
  double _qw;

  geometry_msgs::msg::PoseStamped toMsg() const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = _frame.toStdString();
    pose.pose.position.x = _x;
    pose.pose.position.y = _y;
    pose.pose.position.z = _z;
    pose.pose.orientation.x = _qx;
    pose.pose.orientation.y = _qy;
    pose.pose.orientation.z = _qz;
    pose.pose.orientation.w = _qw;

    return pose;
  }
};

Q_DECLARE_METATYPE(RosPose)

#endif  // ROS_QML_PLUGIN__ROS_TYPES_HPP_
