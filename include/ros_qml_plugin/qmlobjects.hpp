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

#ifndef ROS_QML_PLUGIN__QMLOBJECTS_HPP_
#define ROS_QML_PLUGIN__QMLOBJECTS_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <QObject>
#include <QQuickItem>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "ros_qml_plugin/qobject_ros2.hpp"

//  /**
//   * @brief A QtQuick item that follows a ROS pose published on a topic
//   'topic'.
//   *
//   * The scaling between the ROS pose coordinates (in meters) and the QML
//   pixels
//   * can be set with the property 'pixelscale': pixels = meters / pixelscale
//   *
//   * The Z value of the pose is not directly used (as QML is 2D!), but can be
//   read
//   * from the property 'zvalue'.
//   */
//  class RosPoseSubscriber : public QObjectRos2 {
//    Q_OBJECT
//    Q_PROPERTY(bool position MEMBER _position NOTIFY onPositionChanged)
//    Q_PROPERTY(QQuickItem *origin MEMBER _origin)
//    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
//    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
//    Q_PROPERTY(qreal zvalue READ getZValue NOTIFY onZValueChanged)
//
//  public:
//    RosPoseSubscriber();
//
//    virtual ~RosPoseSubscriber() {}
//
//    void setTopic(QString topic);
//    qreal getZValue() { return _zvalue; }
//
//    void onIncomingPose(const geometry_msgs::msg::PoseStamped &);
//
//  private slots:
//    void updatePos(double x, double y, double z, double rotation);
//
//  signals:
//    void onPositionChanged();
//    void onZValueChanged();
//
//    void onMsgReceived(double x, double y, double z, double rotation);
//
//  private:
//    QString _topic;
//
//    QQuickItem *_origin;
//    double _pixel2meter;
//
//    bool _position; // not really used, but required tfor 'onPositionChanged'
//    to
//                    // be valid in QML
//
//    qreal _zvalue;
//
//    // ros::NodeHandle _node;
//    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
//        _incoming_poses;
//  };
//
//  /**
//   * @brief A QtQuick item that publish a ROS pose on a topic 'topic'.
//   *
//   * The scaling between the ROS pose coordinates (in meters) and the QML
//   pixels
//   * can be set with the property 'pixelscale': pixels = meters / pixelscale
//   *
//   * The Z value of the pose is not directly used (as QML is 2D!), but can be
//   sent
//   * from the property 'zvalue'.
//   */
//  class RosPosePublisher : public QObjectRos2 {
//    Q_OBJECT
//    Q_PROPERTY(QQuickItem *target WRITE setTarget MEMBER _target)
//    Q_PROPERTY(QQuickItem *origin MEMBER _origin)
//    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
//    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
//    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
//
//  public:
//    RosPosePublisher();
//    virtual ~RosPosePublisher() {}
//
//    Q_INVOKABLE void publish();
//
//    void setTopic(QString topic);
//    void setTarget(QQuickItem *target);
//    void setFrame(QString frame);
//
//  private:
//    QString _topic;
//    QQuickItem *_target;
//    QQuickItem *_origin;
//    QString _frame;
//
//    int _width;
//    int _height;
//
//    qreal _pixel2meter;
//
//    // ros::NodeHandle _node;
//    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _publisher;
//  };

/**
 * @brief A QtQuick item that follows a ROS String published on a topic 'topic'.
 */
class RosStringSubscriber : public QObjectRos2
{
  Q_OBJECT
  Q_PROPERTY(QString text MEMBER _text NOTIFY onTextChanged)
  Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:
  RosStringSubscriber() {}

  virtual ~RosStringSubscriber() {}

  void setTopic(QString topic);
  void onIncomingString(const std_msgs::msg::String & str);

signals:
  void onTextChanged();

private:
  QString _topic;
  QString _text;

  // ros::NodeHandle _node;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _subscriber;
};

/**
 * @brief A QtQuick item that publish a ROS string on a topic 'topic'.
 *
 */
class RosStringPublisher : public QObjectRos2
{
  Q_OBJECT
  Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
  Q_PROPERTY(QString text WRITE setText MEMBER _text)

public:
  RosStringPublisher() {}
  virtual ~RosStringPublisher() {}

  void setTopic(QString topic);
  void setText(QString text);
  Q_INVOKABLE void publish();

private:
  QString _topic;
  QString _text;
  // ros::NodeHandle _node;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
};

//  /**
//   * @brief A QtQuick item that follows a ROS TF frame.
//   *
//   * The TF frame is first transformed into the provided ROS 'parentframe'
//   * frame, and the (x,y) coordinates of the resulting frame are used to place
//   * the QtQuick item wrt to the 'origin' item (or (0,0) if 'origin' is not
//   set).
//   *
//   * The scaling between the ROS coordinates (in meters) and the QML pixels
//   can
//   * be set with the property 'pixelscale': pixels = meters / pixelscale
//   *
//   * The Z value of the frame is not directly used (as QML is 2D!), but can be
//   * read from the property 'zvalue'.
//   */
//  class TFListener : public QObjectRos2 {
//    Q_OBJECT
//    Q_PROPERTY(bool position MEMBER _position NOTIFY onPositionChanged)
//    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
//    Q_PROPERTY(QString parentframe WRITE setParentFrame MEMBER _parentframe)
//    Q_PROPERTY(QQuickItem *origin MEMBER _origin)
//    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
//    Q_PROPERTY(qreal zvalue READ getZValue NOTIFY onZValueChanged)
//
//  public:
//    TFListener();
//
//    virtual ~TFListener();
//
//    void setFrame(QString topic);
//    void setParentFrame(QString topic);
//    qreal getZValue() { return _zvalue; }
//
//    void onIncomingPose(const geometry_msgs::msg::PoseStamped &);
//
//  private slots:
//    void updatePos(double x, double y, double z, double rotation);
//
//  signals:
//    void onPositionChanged();
//    void onZValueChanged();
//
//    void onMsgReceived(double x, double y, double z, double rotation);
//
//  private:
//    bool _initialized;
//    bool _active;
//    bool _running;
//    std::thread _listener_thread;
//
//    void
//    listen(); // method ran in the thread that actually listen to the TF
//    updates
//
//    QString _frame;
//    QString _parentframe;
//
//    QQuickItem *_origin;
//    double _pixel2meter;
//
//    bool _position; // not really used, but required tfor 'onPositionChanged'
//    to
//                    // be valid in QML
//
//    qreal _zvalue;
//
//    // ros::NodeHandle _node;
//    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
//    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//  };
//
//  /**
//   * @brief A QML Item that broadcast its target's pose to TF
//   */
//  class TFBroadcaster : public QObjectRos2 {
//    Q_OBJECT
//    Q_PROPERTY(bool active MEMBER _active)
//    Q_PROPERTY(QQuickItem *target WRITE setTarget MEMBER _target)
//    Q_PROPERTY(QQuickItem *origin MEMBER _origin)
//    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
//    Q_PROPERTY(QString parentframe WRITE setParentFrame MEMBER _parentframe)
//    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
//    Q_PROPERTY(double zoffset MEMBER _zoffset)
//
//  public:
//    TFBroadcaster();
//
//    virtual ~TFBroadcaster();
//
//    void setTarget(QQuickItem *target);
//    void setFrame(QString frame);
//    void setParentFrame(QString frame);
//
//  private:
//    void tfPublisher();
//
//    bool _initialized;
//    bool _active;
//    bool _running;
//    std::thread _broadcaster_thread;
//
//    QQuickItem *_target;
//    QQuickItem *_origin;
//    QString _frame;
//    QString _parentframe;
//
//    qreal _pixel2meter;
//    qreal _zoffset;
//
//    // ros::NodeHandle _node;
//    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//  };
//
//  /**
//   * @brief The ImagePublisher class provides a QML object that publishes a
//   QImage
//   * on a ROS topic (set it with the 'topic' property). The QML property
//   'target'
//   * must refer to a QML image. The property 'frame' should be set to the
//   desired
//   * ROS frame.
//   *
//   * The image is published everytime the method 'publish()' is called.
//   *
//   * The size of the image can be set with the property 'width' and 'height'.
//   By
//   * default, the actual size of the image item is used.
//   *
//   * The image can be published on a latched topic by setting `latched: True`
//   (by
//   * default, not latched).
//   *
//   * The property 'pixelscale' is used to compute the (virtual) focal length:
//   f =
//   * 1/pixelscale.  This can be used to convert the image's pixels into meters
//   in
//   * the ROS code: 1 meter = 1 pixel * 1/f
//   */
//  class ImagePublisher : public QObjectRos2 {
//
//    Q_OBJECT
//    Q_PROPERTY(bool active MEMBER _active)
//    Q_PROPERTY(QQuickItem *target WRITE setTarget MEMBER _target)
//    Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)
//    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
//    Q_PROPERTY(bool latched WRITE setLatched MEMBER _latched)
//    Q_PROPERTY(int width MEMBER _width)
//    Q_PROPERTY(int height MEMBER _height)
//    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
//
//  public:
//    ImagePublisher();
//
//    virtual ~ImagePublisher() {}
//
//    void setTarget(QQuickItem *target);
//    void setFrame(QString frame);
//    void setTopic(QString topic);
//    void setLatched(bool latched);
//
//    Q_INVOKABLE void publish();
//
//  private:
//    bool _active;
//
//    sensor_msgs::msg::CameraInfo
//    makeCameraInfo(const sensor_msgs::msg::Image &img);
//    void _rospublish(const QImage &);
//
//    QQuickItem *_target;
//
//    QString _topic;
//    QString _frame;
//    bool _latched;
//
//    int _width;
//    int _height;
//    qreal _pixel2meter;
//
//    // ros::NodeHandle _node;
//    image_transport::ImageTransport _it;
//    image_transport::CameraPublisher _publisher;
//  };
//
//  /**
//   * @brief The FootprintsPublisher class provides a QML object that publishes
//   on
//   * a *latched topic* (/footprints) the 2D bounding boxes of each of its
//   * 'targets'. QML items in 'targets' must have a property 'boundingbox' that
//   * contains a Box2D fixture.
//   */
//  class FootprintsPublisher : public QObjectRos2 {
//
//    Q_OBJECT
//    Q_PROPERTY(QVariantList targets WRITE setTargets MEMBER _targets)
//    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)
//
//  public:
//    static const QString topic;
//
//    FootprintsPublisher();
//
//    virtual ~FootprintsPublisher() {}
//
//    void setTargets(QVariantList targets);
//
//  private:
//    QVariantList _targets;
//
//    qreal _pixel2meter;
//
//    // ros::NodeHandle _node;
//    // ros::Publisher _publisher;
//  };

/**
 * @brief The RosSignal class provides a QML object that publishes on a
 * configurable topic an empty message (ie, a signal) every time signal() is
 * called.
 */
class RosSignal : public QObjectRos2
{
  Q_OBJECT
  Q_PROPERTY(QString topic WRITE setTopic MEMBER _topic)

public:
  RosSignal() {}

  virtual ~RosSignal() {}

  void setTopic(QString topic);

  Q_INVOKABLE void signal();

  void onIncomingSignal(const std_msgs::msg::Empty);

signals:
  void triggered();

private:
  QString _topic;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr _publisher;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _subscriber;
};

#endif  // ROS_QML_PLUGIN__QMLOBJECTS_HPP_
