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

#ifndef IMAGE_PROVIDER_HPP_
#define IMAGE_PROVIDER_HPP_

#include <qquickimageprovider.h>

#include <QImage>
#include <QSize>
#include <QString>
#include <memory>
#include <string>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

class RosImageProvider : public QQuickImageProvider
{
public:
  RosImageProvider();

  QImage requestImage(
    const QString & id, QSize * size,
    const QSize & requestedSize);

  ImageType imageType() const override {return QQmlImageProviderBase::Image;}

private:
  std::string _topic;

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);

  std::shared_ptr<image_transport::Subscriber> _sub;

  QImage _last_image;
};

#endif  // IMAGE_PROVIDER_HPP_
