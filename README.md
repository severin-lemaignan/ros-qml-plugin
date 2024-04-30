ROS 2 QML plugin
================

![Screenshot of the sample app](doc/screenshot-sample-app.png)

Requirements
------------

- `qt5`. On Debian/Ubuntu: `apt install qmake qt5-default qtdeclarative5-dev`
- ROS (tested with ROS humble. Check the `master` branch for kinectic and noetic support).

*Note that this has only been tested on Linux, and would likely require
significant work to get it to work on a different operating system.*

Installation
------------

You can install the plugin as any ROS 2 package, using `colcon`.
If you then source your ROS 2 environment, any qml application will be able to
use the plugin.

To compile:

```bash
> rosdep install --from-paths src --ignore-src -y # install dependencies
> colcon build --packages-select ros_qml_plugin
```

General Usage
-------------

**Important: always launch QtCreator from the command-line! otherwise, your ROS
configuration will not be set up, and QtCreator won't find the ROS libraries.**

In your QML files, import `Ros 2.0`:

```qml

import Ros 2.0
```

### Working 'Hello World' example

This example creates a blank window. If you click anywhere onto the window, the
string `Hello world` is published on the topic `/hello`:

```qml
import QtQuick 2.12
import QtQuick.Window 2.12

import Ros 2.0

Window {
    visible: true
    width: 640
    height: 480
    title: qsTr("Hello World")

    StringTopic{
        id: hello_publisher
        topic: "hello"

    }

    MouseArea {
        anchors.fill: parent
        onClicked: {
            hello_publisher.value = "Hello world"
        }

    }

}
```

Supported ROS features
----------------------

Check [sample.qml](examples/sample.qml) for a complete example.
You can actually test it by running `qmlscene examples/sample.qml` (cf
screenshot above).

Supports:

- [displaying a ROS image topic](#displaying-ros-image-topics)
- setting/reading ROS 2 parameters (`RosParam`) of basic type (string, int,
  float, bool). You must set the property `name` to the parameter name. If you
  also set the `node` property, the parameter will be set on that node,
  otherwise it will be set on the QML node itself.
- bi-directional event signaling (``RosSignal``) by sending an `Empty` message
  on a specfic topic
- publish and subscribe to string, int16, float32 and bool topics
  (`StringTopic`, `IntTopic`, `FloatTopic`, `BoolTopic`).
  To publish, set the `value` property. To subscribe, use the `onMessageReceived` signal.


### Displaying ROS image topics

This uses a special QML ``ImageProvider`` to read images from a ROS topic. Specify the topic using: `img.source = "image://rosimage/<your topic>"`.

Typical usage, that refreshes the image at 20Hz:

```qml
import Ros 2.0

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
```

note that only a limited set of image formats are supported (`rgb888`, `bgr888`, `rgba8888`, `mono8`, `mono16`).



Similar projects
----------------

- https://github.com/StefanFabian/qml_ros2_plugin: similar project, with a focus
  on lower-level access to ROS 2 topics, actions, services. More generic, but
  slightly more complex to use.

