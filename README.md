ROS QML plugin
==============

Supports:

- publishing the pose of QML items as TF frames
- publishing QML items as ROS images
- subscribing to ROS poses to move a QML item
- signaling events by sending an `Empty` message on a specfic topic

When used in conjunction with QML `Box2D` plugin, it can also publish the
(Box2D) bounding boxes of items as polygons.

Requirements
------------

- `qt5`
- ROS (tested with ROS kinectic, but should work with other versions as well)

Installation
------------

The following commands compile and install the QML plugin in the QML dir,
making it available to any QML application.

```
> mkdir build
> cd build
> qmake ..
> make
> make install
```

Known Issue
-----------

Ros has a known error in the pc files, lib are specified as -l:/path/libname.so. So the "-l:" should be removed. This can be done by updating the pc files in ros:

```
> cd /opt/ros/kinetic/lib/pkgconfig/
> sudo sed -i "s/-l://g" *
```

Usage
-----

Call `ros::init` in your `main.cpp`:

```cpp
//...
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv,"your_node_name");

    QGuiApplication app(argc, argv);

    //...
}
```

Then, in your QML files:

```qml

import Ros 1.0

Item {
  id: test

  //...

 TFBroadcaster {
    target: test
    frame: "test_frame"
 }

 Image {
    //...

    ImagePublisher {
      id: imgPublisher
      target: parent
      topic: "/qmlapp/image"
      frame: "/map"
      width: 800
      height: 600
    }

    onPaintedGeometryChanged: imgPublisher.publish()

 }

}
```

### Displaying ROS image topics

This uses a QML ImageProvider. Specify the topic using: `img.source = "image://rosimage/<your topic>"`.

Typical usage, that refreshes the image at 20Hz:

```qml
import Ros 1.0

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

### RosPose

`RosPose` is a QML item whose position can be updated from a ROS `Pose` message.

Only the `pose.x` and `pose.y` are used. Use `pixelscale`
to set the conversion factor between pixels and meters (see example below).

The (`x`, `y`) coordinates are relative to the QML item `origin`.

The following example uses poses published on the `/poses` topic to move a blue
rectangle on the screen:

```qml

Window {

    RosPose {
        id: mypose
        topic: "/poses"
        origin: parent
        pixelscale: 200 // 200 pixels = 1 meter

        Rect {
            width: 10
            height: 10
            color: "blue"
        }
    }
}
```

You can as well use `onPositionChanged` to react to position changes.
