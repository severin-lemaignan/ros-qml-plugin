// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#include "ros_qml_plugin/ros2.hpp"

#include <QCoreApplication>
#include <QJSEngine>
#include <thread>

Ros2Qml & Ros2Qml::getInstance()
{
  static Ros2Qml instance;
  return instance;
}

Ros2Qml::Ros2Qml() {count_wrappers = 0;}

bool Ros2Qml::isInitialized() const {return context_ != nullptr;}

void Ros2Qml::init(const QString & name, quint32 options)
{
  const QStringList & arguments = QCoreApplication::arguments();
  init(name, arguments, options);
}

void Ros2Qml::init(const QString & name, const QStringList & argv, quint32)
{
  if (context_ != nullptr) {
    // TODO(SLE): QML_ROS2_PLUGIN_WARN(
    //     "Was already initialized. Second call to init ignored.");
    return;
  }
  //  std::this_thread::sleep_for(std::chrono::seconds(10));
  int argc = argv.size();
  char ** cargv = new char *[argc];
  for (int i = 0; i < argv.size(); ++i) {
    cargv[i] = new char[argv[i].length() + 1];
    std::string string = argv[i].toStdString();
    std::copy(string.begin(), string.end(), cargv[i]);
  }
  context_ = rclcpp::Context::make_shared();
  context_->init(argc, cargv); // TODO(upstream): init options
  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  node_ = rclcpp::Node::make_shared(
    name.toStdString(),
    node_options);   // TODO(upstream): namespace and init options

  it_ = std::make_shared<image_transport::ImageTransport>(node_);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  // StaticSingleThreadedExecutor may be a bit faster but will keep a reference
  // to the subscription and therefore not unsubscribe if the subscription is
  // reset.
  auto executor =
    rclcpp::executors::SingleThreadedExecutor::make_unique(executor_options);
  executor->add_node(node_);
  for (int i = 0; i < argv.size(); ++i) {
    delete[] cargv[i];
  }
  delete[] cargv;

  executor_thread_ =
    std::thread([executor = std::move(executor)]() {executor->spin();});

  emit initialized();
  std::cout << "ROS 2 initialized." << std::endl;
  // TODO(SLE): QML_ROS2_PLUGIN_DEBUG("QML Ros2 initialized.");
}

bool Ros2Qml::ok() const {return rclcpp::ok();}

void Ros2Qml::registerDependant() {++count_wrappers;}

void Ros2Qml::unregisterDependant()
{
  int count = --count_wrappers;
  if (count == 0) {
    // TODO(SLE): QML_ROS2_PLUGIN_DEBUG("No dependants left. QML Ros2 shutting
    // down.");
    rclcpp::shutdown(
      context_, "All dependants unregistered, usually that "
      "means the application is exiting.");
    emit shutdown();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    node_.reset();
    context_.reset();
    // TODO(SLE): QML_ROS2_PLUGIN_DEBUG("QML Ros2 shut down.");
  } else if (count < 0) {
    // TODO(SLE): QML_ROS2_PLUGIN_WARN("Stop spinning was called more often than
    // start "
    // "spinning! This is a bug!");
    ++count_wrappers;
  }
}

std::shared_ptr<rclcpp::Node> Ros2Qml::node() {return node_;}
std::shared_ptr<image_transport::ImageTransport> Ros2Qml::image_transport()
{
  return it_;
}
