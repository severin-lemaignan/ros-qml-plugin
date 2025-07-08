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

#include <QImage>
#include <QPainter>
#include <iostream>
#include <memory>

#include "image_provider.hpp"
#include "ros_qml_plugin/ros2.hpp"

using std::placeholders::_1;

RosImageProvider::RosImageProvider()
: QQuickImageProvider(QQuickImageProvider::Pixmap),
  //    _it(ros::NodeHandle()),
  _last_image(QImage(10, 10, QImage::Format_RGB888))
{
  _last_image.fill(QColor("black").rgba());
}

void RosImageProvider::imageCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  if (msg->encoding == "rgb8") {
    _last_image = QImage(msg->width, msg->height, QImage::Format_RGB888);
  } else if (msg->encoding == "bgr8") {
    _last_image = QImage(msg->width, msg->height, QImage::Format_BGR888);
  } else if (msg->encoding == "rgba8") {
    _last_image = QImage(msg->width, msg->height, QImage::Format_RGBA8888);
  } else if (msg->encoding == "mono8") {
    _last_image = QImage(msg->width, msg->height, QImage::Format_Grayscale8);

  } else if (msg->encoding == "mono16") {
    _last_image = QImage(msg->width, msg->height, QImage::Format_Grayscale16);
  } else {
    std::cerr << "Unsupported image encoding: " << msg->encoding
              << " (supported: rgb8, bgr8, rgba8, mono8, mono16)" << std::endl;
    return;
  }

  memcpy(_last_image.bits(), msg->data.data(), _last_image.sizeInBytes());
}

QImage RosImageProvider::requestImage(
  const QString & id, QSize * size,
  const QSize & requestedSize)
{

  // remove '?' and everything after it, if present
  QString topic = id;
  int questionMarkIndex = topic.indexOf('?');
  if (questionMarkIndex != -1) {
    topic = topic.left(questionMarkIndex);
  }

  // if topic does not start with '/', prepend it
  if (!topic.startsWith('/')) {
    topic.prepend('/');
  }


  if (_topic != topic.toStdString()) {
    _topic = topic.toStdString();
    std::cout << "Subscribing to image topic " << _topic << std::endl;

    auto node = Ros2Qml::getInstance().node();

    _sub = std::make_shared<image_transport::Subscriber>(
      image_transport::create_subscription(
        node.get(),
        _topic,
        std::bind(&RosImageProvider::imageCallback, this, std::placeholders::_1),
        "compressed",
        rmw_qos_profile_sensor_data
      )
    );
  }

  // std::cout << "Image requested" << std::endl;

  QImage result;

  // std::cout << "Last image: " << _last_image.width() << "x" <<
  //  _last_image.height() << ")" << std::endl;

  if (requestedSize.isValid()) {
    // std::cout << "(resizing image to " << requestedSize.width() << "x" <<
    //  requestedSize.height() << ")" << std::endl;
    result = _last_image.scaled(requestedSize, Qt::KeepAspectRatio);
  } else {
    // std::cout << "(not resizing image)" << std::endl;
    result = _last_image;
  }

  *size = result.size();
  return result;
}
