^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_qml_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.21.6 (2025-08-29)
-------------------
* avoid ROS segfault by returning early if ROS2 not ready
* Contributors: Séverin Lemaignan

2.21.5 (2025-08-29)
-------------------
* partially revert 857f033af7 to still use the underlying navigate_to_pose skill
  navigate_to_pose has been taught to deal with missing reference frames; for now we can use that.
  Keep the QML 'navigate' name in the API to be future-proof (when the proper 'navigate' skill will be ready)
* Contributors: Séverin Lemaignan

2.21.4 (2025-08-28)
-------------------
* skill: navigate_to_pose -> navigate
* Contributors: Séverin Lemaignan

2.21.3 (2025-08-28)
-------------------
* linting
* allow Ros.point and Ros.pose without explicit frame
* Contributors: Séverin Lemaignan

2.21.2 (2025-08-05)
-------------------
* chat skill: configure the prompt in th default role
* Contributors: Séverin Lemaignan

2.21.1 (2025-07-31)
-------------------
* remove un-needed dependency on ament_cmake_pal
* Contributors: Séverin Lemaignan

2.21.0 (2025-07-30)
-------------------
* impl Chat skill
* enable backward_ros to get stacktraces when plugin crashing
* expose 'livespeech' topic to eg simulate someone speaking
* Contributors: Séverin Lemaignan

2.20.1 (2025-07-29)
-------------------
* service: setbool: ensure we notify when value changed
* Contributors: Séverin Lemaignan

2.20.0 (2025-07-18)
-------------------
* expose the navigate_to_pose skill
* Contributors: Séverin Lemaignan

2.19.2 (2025-07-09)
-------------------
* linting
* Contributors: Séverin Lemaignan

2.19.1 (2025-07-08)
-------------------
* ROS image provider: allows (and trims) '?...' after the topic name
  Useful to force QML to reload the image source
* Contributors: Séverin Lemaignan

2.19.0 (2025-07-03)
-------------------
* linting
* improve logging
* change how the Ros singleton is created, to make it possible to import it from JS modules in QML
* improve API of look_at skill
  ROS PointStamped are created and passed with: Ros.point(frame,x,y,z)
* disable float-equal warnings -- Qt MOC generates code that triggers that warning
* implement the 'look_at' skill
* Contributors: Séverin Lemaignan

2.18.1 (2025-07-02)
-------------------
* linter
* Contributors: Séverin Lemaignan

2.18.0 (2025-06-26)
-------------------
* expose SetExpressionSkill
* Contributors: Séverin Lemaignan

2.17.0 (2025-06-23)
-------------------
* {/say -- /skill/say}
* Contributors: Séverin Lemaignan

2.16.0 (2025-05-30)
-------------------
* linting
* Say skill: expose a 'say' method
* Contributors: Séverin Lemaignan

2.15.1 (2025-05-30)
-------------------
* add image_transport_plugins as an exec_depend
  otherwise, the QML engine will crash at runtime, as rosimage provider expects compressed video streams
* Contributors: Séverin Lemaignan

2.15.0 (2025-05-28)
-------------------
* add support for the Say skill
* Contributors: Séverin Lemaignan

2.14.2 (2025-05-28)
-------------------
* subscribe to images using BEST_EFFORT QoS
* [doc] update the sample.qml file to use current API
* Contributors: Séverin Lemaignan

2.14.1 (2025-05-20)
-------------------
* when setting a topic value from within the QML, emit onValueCHanged
* Contributors: Séverin Lemaignan

2.14.0 (2025-05-16)
-------------------
* Create the publishers and subscribers only if the isPublisher or isSubscriber are true. Delete them once these variables are set false
* Contributors: ferrangebelli

2.13.0 (2025-03-31)
-------------------
* expose the properties isSubscriber/isPublisher to control the direction of Topics
  By default, topics are still bidirectional
* Contributors: Séverin Lemaignan

2.12.0 (2025-03-31)
-------------------
* intent topic: add support for 'source' field
* Contributors: Séverin Lemaignan

2.11.0 (2025-03-26)
-------------------
* added int32 subscriber
* Contributors: ferrangebelli

2.10.0 (2025-01-16)
-------------------
* add support to set custom ROS node name
* add SetBoolService to create ROS SetBool service from QML
* Contributors: Séverin Lemaignan

2.9.0 (2024-12-17)
------------------
* add SetUiFragment service
* Contributors: Séverin Lemaignan

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
