^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_qml_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.8.0 (2024-12-12)
------------------
* intent topic: add constants for all intent types
* expose Intent.modality to the IntentTopic
* Contributors: Séverin Lemaignan

2.7.0 (2024-10-29)
------------------
* setting remote parameter if it is set in qml on initialization
* Add goal rejected signal for actions, and fix tests
* Service and action not waiting for response, but triggering a callback
* Get remote parameter on initialization
* Uncrustify applied to pass tests
* Added ros services and actions to the plugin
* get language service first implementation
* Contributors: Luka Juricic, ferrangebelli

2.6.1 (2024-10-15)
------------------
* linting
* Contributors: Séverin Lemaignan

2.6.0 (2024-10-15)
------------------
* add support for string array parameters
* Contributors: Séverin Lemaignan

2.5.0 (2024-10-15)
------------------
* add support for hri_actions_msgs::Intent topics
* linting
* Contributors: Séverin Lemaignan

2.4.1 (2024-10-15)
------------------
* [minor] linting
* Contributors: Séverin Lemaignan

2.4.0 (2024-08-01)
------------------
* add support for ClosedCaptions
  While here, removed support for LiveSpeech
* Contributors: Séverin Lemaignan

2.3.1 (2024-08-01)
------------------
* fix remote parameter setting/reading
* Contributors: Séverin Lemaignan

2.3.0 (2024-08-01)
------------------
* add hooks to configure QML2_IMPORT_PATH
* Contributors: Séverin Lemaignan

2.2.1 (2024-07-01)
------------------
* [linter] run ament_uncrustify
* Contributors: Séverin Lemaignan

2.2.0 (2024-05-02)
------------------
* add subscriber/publisher for hri_msgs/Expression and hri_msgs/LiveSpeech
* code layout refactoring
  moved the different QML objects in their own cpp/hpp
* Contributors: Séverin Lemaignan

2.1.0 (2024-04-30)
------------------
* [doc] update README to reflect what the plugin can actually do
* create a generic RosTopic class
  + expose IntTopic, FloatTopic, BoolTopic, String topic
  Remove (now useless) RosStringPublisher/Subscriber
* add a RosTopicInt publisher/subscriber
* add a RosParam qml object to set/get ROS2 parameters on the current node
* minor Qt styling change
* Contributors: Séverin Lemaignan

2.0.0 (2024-04-26)
------------------
* bump version in prep of first ROS2 release
* add dep on ament_cmake_auto
* handle more image formats
* initial steps towards ROS2 port
  The whole source code now behaves as a regular ROS 2 package. It can be
  compile with colcon.
  Currently working QML objects:
  - RosStringPublisher
  - RosStringSubscriber
  - RosSignal
  - image subscribing via source: "image://rosimage/image_raw"
* More documentation
* More detailled documentation
* Added missing dep. on visualization_msgs
* [doc] list required deb packages to compile the plugin
* Significantly improved documentation
* [doc] Minor update
* Fix many minor compilation warnings
* ImagePublisher: Support latched image topic
* Added support for reading and displaying ROS image topics
  This uses a QML ImageProvider. Specify the topic using:
  img.source = "image://rosimage/<your topic>"
  Typical usage:
  Image {
  id: img
  cache: false
  anchors.fill: parent
  source: "image://rosimage/v4l/camera/image_raw"
  Timer {
  interval: 50
  repeat: true
  running: true
  onTriggered: { img.source = ""; img.source = "image://rosimage/v4l/camera/image_raw" }
  }
  }
* add RosString publisher and subscriber
* Added listener to signal
* change RosPose to RosPoseListener and add RosPosePublisher with support for rotation
* Make sure we do not send TF transforms with NaNs
* Fixed a severe memory leak in ImagePublisher
* added support for rotation in update of position
* Update README.md
  Added cd build step
  And update known issues
* Added a 'TFListener' to track a TF frame
  Similar to RosPose, but for TF frames
* RosPose now exposes the 'z' value of the pose as well
* [ImagePublisher] Publish as well camera info, using pixel2meter to provide a 'virtual' focal length
* Added support to publish QML items as ROS images
* Add support to set the ropic of RosPose
  While here, update doc
* Added a property 'zoffset' to TfBroadcaster to the a custom Z value (0 by default)
* Do not crash if a non-object it passed in Footprints targets
* Added the 'RosSignal' type to publish an Empty on a topic whenever 'signal()' is called from QML
* Added a property 'active' to TfBroadcaster to inhibit/activate broadcasting
* Added a 'footprint' publisher, that publishes items' bounding boxes in a ROS MarkerArray
* Listen by default on the topic 'poses' for poses
* Ensure we always use scene coordinates
* Set origin/pixel2meter conversion factors when receiving ROS pose
* The ROS plugin can now listen to topics and send updates to the QT GUI thread
* Added a README
* Fixed scaling/rotation issues. User might now specify an item used as origin
* TF transforms are now successfully published by the 'TFBroadcaster' QML item
* First actual connection to ROS
* Initial import of a QML plugin to export QML Item poses as ROS frames
* Contributors: Emmanuel Senft, EmmanuelSenft, Séverin Lemaignan
