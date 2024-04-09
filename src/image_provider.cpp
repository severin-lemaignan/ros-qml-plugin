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

  _last_image = QImage(msg->width, msg->height, QImage::Format_RGB888);
  memcpy(_last_image.bits(), msg->data.data(), _last_image.sizeInBytes());
}

QImage RosImageProvider::requestImage(
  const QString & id, QSize * size,
  const QSize & requestedSize)
{
  if (_topic != id.toStdString()) {
    _topic = id.toStdString();
    std::cout << "Subscribing to image topic " << _topic << std::endl;

    auto it = Ros2Qml::getInstance().image_transport();

    _sub = std::make_shared<image_transport::Subscriber>(
      it->subscribe(_topic, 1, &RosImageProvider::imageCallback, this));
  }

  // cout << "Image requested" << endl;

  QImage result;

  // cout << "Last image: " << _last_image.width() << "x" <<
  // _last_image.height()
  //      << ")" << endl;

  if (requestedSize.isValid()) {
    result = _last_image.scaled(requestedSize, Qt::KeepAspectRatio);
  } else {
    result = _last_image;
  }

  *size = result.size();
  return result;
}
