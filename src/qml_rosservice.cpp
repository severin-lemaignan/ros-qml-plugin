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

#include "ros_qml_plugin/qml_rosservice.hpp"

GetLocalesService::GetLocalesService()
: RosService<i18n_msgs::srv::GetLocales>("i18n_manager/") {}

GetLocalesService::~GetLocalesService() {}

bool GetLocalesService::request()
{
  cancel();

  auto success = RosService::request(
    std::make_shared<i18n_msgs::srv::GetLocales::Request>(),
    [this](SharedFutureResponse future) {
      QVector<QString> locales;
      if (future.valid()) {
        if (auto response = future.get()) {
          for (const auto & locale : response->locales) {
            locales.push_back(QString::fromStdString(locale));
          }
          emit onResponse(true, locales);
        }
      }
      emit onResponse(false, locales);
    },
    rclcpp::Duration::from_seconds(1));

  return success;
}
