// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full
// license information.

#ifndef ROS_QML_PLUGIN__ROS2_HPP_
#define ROS_QML_PLUGIN__ROS2_HPP_

#include <QJSValue>
#include <QObject>
#include <QTimer>
#include <memory>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

class Ros2Qml : public QObject
{
  Q_OBJECT

private:
  Ros2Qml();

public:
  static Ros2Qml & getInstance();

  Ros2Qml(const Ros2Qml &) = delete;

  void operator=(const Ros2Qml &) = delete;

  /*!
   * Checks whether ROS is initialized.
   * @return True if ROS is initialized, false otherwise.
   */
  bool isInitialized() const;

  /*!
   * Initializes the ros node with the given name and the command line arguments
   * passed from the command line.
   * @param name The name of the ROS node.
   * @param options The options passed to ROS, see
   * ros_init_options::Ros2InitOption.
   */
  void init(const QString & name, quint32 options = 0);

  /*!
   * Initializes the ros node with the given args.
   * @param name The name of the ROS node.
   * @param args The args that are passed to ROS. Normally, these would be the
   * command line arguments see init(const QString &, quint32)
   * @param options The options passed to ROS, see
   * ros_init_options::Ros2InitOption.
   */
  void init(const QString & name, const QStringList & args, quint32 options = 0);

  /*!
   * Can be used to query the state of ROS.
   * @return False if it's time to exit, true if still ok.
   */
  bool ok() const;

  //! Increases the dependant counter.
  void registerDependant();

  //! Decreases the dependant counter and if it gets to 0, frees all memory.
  void unregisterDependant();

  std::shared_ptr<rclcpp::Node> node();
  std::shared_ptr<image_transport::ImageTransport> image_transport();

signals:
  //! Emitted once when ROS was initialized.
  void initialized();

  //! Emitted when this ROS node was shut down and it is time to exit.
  void shutdown();

protected slots:
  //  void checkShutdown();

private:
  //  void onInitialized();

  std::thread executor_thread_;
  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  std::atomic<int> count_wrappers;
};

#endif  // ROS_QML_PLUGIN__ROS2_HPP_
