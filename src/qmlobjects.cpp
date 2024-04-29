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

#include <tf2/transform_datatypes.h>

#include <QQuickItemGrabResult>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ros_qml_plugin/qmlobjects.hpp"
#include "ros_qml_plugin/ros2.hpp"

using std::placeholders::_1;

const double EPSILON = 0.5;

//  ////////////////////////////////////////////////////////////////////////////////
//  ///////////////////////////////////////////////////////////////////////////////
//  ///////////////////////////////////////////////////////////////////////////////
//
//  RosPoseSubscriber::RosPoseSubscriber() : _origin(nullptr), _pixel2meter(1) {
//
//    connect(this, SIGNAL(onMsgReceived(double, double, double, double)), this,
//            SLOT(updatePos(double, double, double, double)));
//  }
//
//  void RosPoseSubscriber::onIncomingPose(
//      const geometry_msgs::msg::PoseStamped &pose) {
//    emit onMsgReceived(pose.pose.position.x, pose.pose.position.y,
//                       pose.pose.position.z, 0);
//  }
//
//  void RosPoseSubscriber::updatePos(double x, double y, double z,
//                                    double rotation) {
//    double px, py;
//    if (_origin) {
//      px = x / _pixel2meter + _origin->x();
//      py = -y / _pixel2meter + _origin->y();
//    } else {
//      px = x / _pixel2meter;
//      py = -y / _pixel2meter;
//    }
//
//    if (abs(px - this->x()) > EPSILON || abs(py - this->y()) > EPSILON) {
//
//      setX(px);
//      setY(py);
//
//      emit onPositionChanged();
//    }
//
//    auto oldz = _zvalue;
//    _zvalue = z;
//    if (z != oldz)
//      emit onZValueChanged();
//  }
//
//  void RosPoseSubscriber::setTopic(QString topic) {
//
//    //  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();
//    //
//    //  _incoming_poses = _node.subscribe(topic.toStdString(), 1,
//    //                                    &RosPoseSubscriber::onIncomingPose,
//    //                                    this);
//    //
//    _topic = topic;
//  }
//
//  ///////////////////////////////////////////////////////////////////////////////
//  ///////////////////////////////////////////////////////////////////////////////
//  ///////////////////////////////////////////////////////////////////////////////
//
//  RosPosePublisher::RosPosePublisher()
//      : _topic("topic"), _target(nullptr), _origin(nullptr), _frame(""),
//        _width(0), _height(0), _pixel2meter(1) {}
//
//  void RosPosePublisher::setTopic(QString topic) {
//    //  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();
//    //  _publisher = node->create_publisher<geometry_msgs::msg::PoseStamped>(
//    //      topic.toStdString(), 1);
//    _topic = topic;
//  }
//
//  void RosPosePublisher::setTarget(QQuickItem *target) { _target = target; }
//
//  void RosPosePublisher::setFrame(QString frame) { _frame = frame; }
//
//  void RosPosePublisher::publish() {
//    double x, y, theta;
//    if (_origin) {
//      x = _origin->mapFromItem(_target, QPoint(0, 0)).x() * _pixel2meter;
//      y = -_origin->mapFromItem(_target, QPoint(0, 0)).y() * _pixel2meter;
//      theta = (_target->rotation() - _origin->rotation()) * M_PI / 180;
//    } else {
//      x = _target->mapToScene(QPoint(0, 0)).x() * _pixel2meter;
//      y = -_target->mapToScene(QPoint(0, 0)).y() * _pixel2meter;
//      theta = -_target->rotation() * M_PI / 180;
//    }
//
//    geometry_msgs::msg::PoseStamped pose;
//    pose.header.frame_id = _frame.toStdString();
//    pose.header.stamp = ros::Time(0);
//    pose.pose.position.x = x;
//    pose.pose.position.y = y;
//    pose.pose.position.z = 0;
//    tf::Quaternion q;
//    q.setRPY(0, 0, theta);
//    quaternionTFToMsg(q, pose.pose.orientation);
//
//    //  _publisher.publish(pose);
//  }

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void RosStringSubscriber::onIncomingString(const std_msgs::msg::String & str)
{

  // std::cout << "Received string: " << str.data << std::endl;
  setProperty("text", QString::fromStdString(str.data));
}

void RosStringSubscriber::setTopic(QString topic)
{
  if (topic == _topic) {
    return;
  }

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  _subscriber = node->create_subscription<std_msgs::msg::String>(
    topic.toStdString(), 1,
    std::bind(&RosStringSubscriber::onIncomingString, this, _1));

  _topic = topic;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void RosStringPublisher::setTopic(QString topic)
{
  if (topic == _topic) {
    return;
  }

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();
  _publisher =
    node->create_publisher<std_msgs::msg::String>(topic.toStdString(), 1);
  _topic = topic;
}

void RosStringPublisher::setText(QString text)
{
  _text = text;
  publish();
}

void RosStringPublisher::publish()
{
  if (std::string(_publisher->get_topic_name()).empty()) {
    std::cerr << "RosStringPublisher.publish() called without any topic."
              << std::endl;
    return;
  }

  std_msgs::msg::String message;
  message.data = _text.toStdString();

  _publisher->publish(message);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// TFListener::TFListener()
//     : _initialized(false), _active(true), _running(false), _frame(""),
//       _parentframe(""), _origin(nullptr), _pixel2meter(1) {
//
//   connect(this, SIGNAL(onMsgReceived(double, double, double, double)), this,
//           SLOT(updatePos(double, double, double, double)));
// }
//
// TFListener::~TFListener() {
//   if (_running) {
//     _running = false;
//     _listener_thread.join();
//   }
// }
//
// void TFListener::setFrame(QString frame) {
//
//   _frame = frame;
//   if (!_parentframe.isEmpty())
//     _initialized = true;
//
//   if (!_running) {
//     _running = true;
//
//     _listener_thread = std::thread(&TFListener::listen, this);
//   }
// }
//
// void TFListener::setParentFrame(QString frame) {
//
//   _parentframe = frame;
//   if (!_frame.isEmpty())
//     _initialized = true;
//
//   // cout << "Parent frame set to: " << _parentframe.toStdString() << endl;
// }
//
// void TFListener::updatePos(double x, double y, double z, double rotation) {
//   double px, py, theta;
//   if (_origin) {
//     px = x / _pixel2meter + _origin->x();
//     py = -y / _pixel2meter + _origin->y();
//     theta = -(rotation - _origin->rotation()) * 180 / M_PI;
//   } else {
//     px = x / _pixel2meter;
//     py = -y / _pixel2meter;
//   }
//
//   if (abs(px - this->x()) > EPSILON || abs(py - this->y()) > EPSILON ||
//       abs(theta - this->rotation()) > EPSILON) {
//     setX(px);
//     setY(py);
//     setRotation(theta);
//
//     emit onPositionChanged();
//   }
//
//   auto oldz = _zvalue;
//   _zvalue = z;
//   if (z != oldz)
//     emit onZValueChanged();
// }
//
// void TFListener::listen() {
//   while (_running) {
//     if (_initialized && _active) {
//
//       //      tf::StampedTransform transform;
//       //
//       //      try {
//       //        _listener.lookupTransform(_parentframe.toStdString(),
//       //                                  _frame.toStdString(), ros::Time(0),
//       //                                  transform);
//       //        tf::Matrix3x3 m(transform.getRotation());
//       //        double roll, pitch, yaw;
//       //        m.getRPY(roll, pitch, yaw);
//       //        emit onMsgReceived(transform.getOrigin().x(),
//       //        transform.getOrigin().y(),
//       //                           transform.getOrigin().z(), yaw);
//       //      } catch (tf::TransformException ex) {
//       //        ROS_ERROR_THROTTLE(10, "%s", ex.what());
//       //      }
//     }
//     this_thread::sleep_for(chrono::milliseconds(50));
//   }
// }
//
// ///////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
//
// TFBroadcaster::TFBroadcaster()
//     : _initialized(false), _active(true), _running(false), _target(nullptr),
//       _origin(nullptr), _frame(""), _parentframe(""), _pixel2meter(1),
//       _zoffset(0) {}
//
// TFBroadcaster::~TFBroadcaster() {
//   if (_running) {
//     _running = false;
//     _broadcaster_thread.join();
//   }
// }
//
// void TFBroadcaster::setTarget(QQuickItem *target) {
//
//   _target = target;
//
//   if (!_running) {
//     _running = true;
//
//     _broadcaster_thread = std::thread(&TFBroadcaster::tfPublisher, this);
//   }
// }
//
// void TFBroadcaster::setFrame(QString frame) {
//
//   _frame = frame;
//   if (!_parentframe.isEmpty())
//     _initialized = true;
//
//   // cout << "Frame set to: " << _frame.toStdString() << endl;
// }
//
// void TFBroadcaster::setParentFrame(QString frame) {
//
//   _parentframe = frame;
//   if (!_frame.isEmpty())
//     _initialized = true;
//
//   // cout << "Parent frame set to: " << _parentframe.toStdString() << endl;
// }
//
// void TFBroadcaster::tfPublisher() {
//   while (_running) {
//     if (_initialized && _active) {
//       //      double x, y, theta;
//       //      if (_origin) {
//       //        x = (_target->mapToScene(QPoint(0, 0)).x() -
//       //             _origin->mapToScene(QPoint(0, 0)).x()) *
//       //            _pixel2meter;
//       //        y = -(_target->mapToScene(QPoint(0, 0)).y() -
//       //              _origin->mapToScene(QPoint(0, 0)).y()) *
//       //            _pixel2meter;
//       //        theta = -(_target->rotation() - _origin->rotation()) * M_PI /
//       //        180;
//       //      } else {
//       //        x = _target->mapToScene(QPoint(0, 0)).x() * _pixel2meter;
//       //        y = -_target->mapToScene(QPoint(0, 0)).y() * _pixel2meter;
//       //        theta = -_target->rotation() * M_PI / 180;
//       //      }
//       //
//       //      if (!isnan(x) && !isnan(y) && !isnan(theta)) {
//       //
//       //        tf::Transform transform;
//       //        transform.setOrigin(tf::Vector3(x, y, _zoffset));
//       //
//       //        tf::Quaternion q;
//       //        q.setRPY(0, 0, theta);
//       //        transform.setRotation(q);
//       //        _br.sendTransform(tf::StampedTransform(transform,
//       //        ros::Time::now(),
//       // _parentframe.toStdString(),
//       // _frame.toStdString()));
//       //      }
//     }
//     this_thread::sleep_for(chrono::milliseconds(100));
//   }
// }
//
// ///////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
//
// ImagePublisher::ImagePublisher()
//     : _active(true), _target(nullptr), _topic("image"), _frame(""),
//       _latched(false), _width(0), _height(0), _pixel2meter(1), _it(_node) {
//
//   //  _publisher = _it.advertiseCamera(_topic.toStdString(), 1, _latched);
// }
//
// /* based on
//  * https://github.com/ros-drivers/video_stream_opencv/blob/master/src/video_stream.cpp
//  */
// sensor_msgs::msg::CameraInfo
// ImagePublisher::makeCameraInfo(const sensor_msgs::msg::Image &img) {
//   sensor_msgs::msg::CameraInfo cam_info_msg;
//   cam_info_msg.header.frame_id = img.header.frame_id;
//   // Fill image size
//   cam_info_msg.height = img.height;
//   cam_info_msg.width = img.width;
//   // Add the most common distortion model as sensor_msgs/CameraInfo says
//   cam_info_msg.distortion_model = "plumb_bob";
//   // Don't let distorsion matrix be empty
//   cam_info_msg.D.resize(5, 0.0);
//   // Give a reasonable default intrinsic camera matrix
//   cam_info_msg.K = {{1. / _pixel2meter, 0.0, img.width / 2.0, 0.0,
//                      1. / _pixel2meter, img.height / 2.0, 0.0, 0.0, 1.0}};
//   // Give a reasonable default rectification matrix
//   cam_info_msg.R = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
//   // Give a reasonable default projection matrix
//   cam_info_msg.P = {{1. / _pixel2meter, 0.0, img.width / 2.0, 0.0, 0.0,
//                      1. / _pixel2meter, img.height / 2.0, 0.0, 0.0, 0.0, 1.0,
//                      0.0}};
//   return cam_info_msg;
// }
//
// void ImagePublisher::setTarget(QQuickItem *target) { _target = target; }
//
// void ImagePublisher::publish() {
//
//   // if _width or _height are 0, size is invalid, and grabToImage uses the
//   item
//       // actual size
//       QSize size(_width, _height);
//
//   auto result = _target->grabToImage(size);
//   connect(result.data(), &QQuickItemGrabResult::ready, this, [result, this]()
//   {
//     this->_rospublish(result.data()->image());
//     result.data()->disconnect(); // disconnect the signal, and as a
//     consequence,
//     // free the shared ptr 'result'
//   });
// }
//
// void ImagePublisher::setFrame(QString frame) {
//
//   _frame = frame;
//   // cout << "Frame set to: " << _frame.toStdString() << endl;
// }
//
// void ImagePublisher::setTopic(QString topic) {
//
//   if (topic != _topic) {
//     _topic = topic;
//     //    _publisher.shutdown();
//     //    _publisher = _it.advertiseCamera(_topic.toStdString(), 1,
//     _latched);
//   }
// }
//
// void ImagePublisher::setLatched(bool latched) {
//
//   if (latched != _latched) {
//     _latched = latched;
//     //    _publisher.shutdown();
//     //    _publisher = _it.advertiseCamera(_topic.toStdString(), 1,
//     _latched);
//   }
// }
//
// void ImagePublisher::_rospublish(const QImage &image) {
//
//   if (_active) {
//
//     auto size = image.size();
//     auto img = image.convertToFormat(QImage::Format_RGBA8888);
//
//     sensor_msgs::msg::Image msg;
//     msg.header.frame_id = _frame.toStdString();
//     msg.header.stamp = ros::Time::now();
//     msg.height = size.height();
//     msg.width = size.width();
//     msg.step = img.bytesPerLine();
//     msg.encoding = "rgba8";
//     msg.data.resize(img.byteCount()); // allocate memory
//     memcpy(msg.data.data(), img.constBits(), img.byteCount());
//
//     auto camerainfo = makeCameraInfo(msg);
//     //    _publisher.publish(msg, camerainfo, ros::Time::now());
//   }
// }
//
// ///////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
//
// const QString FootprintsPublisher::topic = "footprints";
//
// FootprintsPublisher::FootprintsPublisher()
//     : _pixel2meter(1),
//       _publisher(_node.advertise<visualization_msgs::msg::MarkerArray>(
//           topic.toStdString(), 1, true)) {}
//
// void FootprintsPublisher::setTargets(QVariantList targets) {
//   visualization_msgs::msg::MarkerArray markers;
//
//   cout << "Setting targets for footprint publishing. Got " <<
//   targets.length()
//        << " of them" << endl;
//   QVariantList::const_iterator i;
//
//   int id = 0;
//
//   for (i = targets.begin(); i != targets.end(); ++i) {
//     auto item = (*i).value<QQuickItem *>();
//
//     if (!item) {
//       cerr << "One of my targets is does not exist! Skipping it." << endl;
//       continue;
//     }
//     auto target = item->property("name").value<QString>().toStdString();
//     auto boundingbox = item->property("boundingbox").value<QObject *>();
//     auto vertices = boundingbox->property("vertices").value<QVariantList>();
//
//     cout << target << endl;
//
//     visualization_msgs::msg::Marker marker;
//     marker.header.frame_id = "/" + target;
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "qml_items_footprints";
//     marker.action = visualization_msgs::msg::Marker::ADD;
//     marker.pose.orientation.w = 1.0;
//
//     marker.id = id++;
//     marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
//     marker.scale.x = 0.005; // width of the visualized line
//     marker.color.b = 1.0;
//     marker.color.a = 1.0;
//
//     QVariantList::const_iterator point_it;
//
//     double bbx = 0, bby = 0;
//
//     for (point_it = vertices.begin(); point_it != vertices.end(); ++point_it)
//     {
//       geometry_msgs::msg::Point p;
//       auto point = (*point_it).value<QPointF>();
//       p.x = point.x() * _pixel2meter;
//       bbx += p.x;
//       p.y = -point.y() * _pixel2meter;
//       bby += p.y;
//       p.z = 0;
//       marker.points.push_back(p);
//     }
//     marker.points.push_back(marker.points.front());
//
//     // compute the center of the bounding box
//     bbx /= vertices.length();
//     bby /= vertices.length();
//
//     for (auto &p : marker.points) {
//       p.x -= bbx;
//       p.y -= bby;
//     }
//
//     markers.markers.push_back(marker);
//   }
//
//   //  _publisher.publish(markers);
// }
//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

void RosSignal::setTopic(QString topic)
{
  //  _publisher = _node.advertise<std_msgs::msg::Empty>(topic.toStdString(),
  //  1); _subscriber = _node.subscribe(topic.toStdString(), 1,
  //                                &RosSignal::onIncomingSignal, this);
  if (topic == _topic) {
    return;
  }

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();

  _subscriber = node->create_subscription<std_msgs::msg::Empty>(
    topic.toStdString(), 1,
    std::bind(&RosSignal::onIncomingSignal, this, _1));

  _publisher =
    node->create_publisher<std_msgs::msg::Empty>(topic.toStdString(), 1);

  _topic = topic;
}

void RosSignal::onIncomingSignal(const std_msgs::msg::Empty /* msg */)
{
  emit triggered();
}

void RosSignal::signal()
{
  if (std::string(_publisher->get_topic_name()).empty()) {
    std::cerr << "RosSignal.signal() called without any topic." << std::endl;
    return;
  }

  _publisher->publish(std_msgs::msg::Empty());
}
