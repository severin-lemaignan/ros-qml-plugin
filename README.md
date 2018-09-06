ROS QML plugin
==============

Supports:

- [publishing the pose of QML items](#tfbroadcaster) as TF frames
- [subscribing to TF frames](#tflistener) to update the pose of QML items
- [subscribe](#rospose)/publish ROS poses to move a QML item
- [publishing QML items as ROS images](#imagepublisher)
- [displaying a ROS image topic](#displaying-ros-image-topics)
- bi-directional event signaling (``RosSignal``) by sending an `Empty` message on a specfic topic
- publish (``RosStringPublisher``)/subscribe (``RosStringSubscriber``) to a string topic and trigger event when a string is received.


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

### Known Issue

ROS has a known error in its `pkgconfig` files (`.pc`) as libs dependencies are
specified as `-l:/path/libname.so`: `-l:` should be removed. This can be done by
updating the `.pc` files in ROS:

```
> cd /opt/ros/kinetic/lib/pkgconfig/
> sudo sed -i "s/-l://g" *
```

General Usage
-------------

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

 //... see below for the list of supported functionalities

}
```


ImagePublisher
--------------

You can publish the content of any QML image as a ROS image using the
``ImagePublisher`` object.

The ImagePublisher class provides a QML object that publishes a QImage on a
ROS topic (set it with the 'topic' property).
The QML property 'target' must refer to a QML image. The property 'frame' should be set to
the desired ROS frame.

The image is published everytime the method 'publish()' is called.

The size of the image can be set with the property 'width' and 'height'. By default, the
actual size of the image item is used.

The image can be published on a latched topic by setting `latched: True` (by
default, not latched).

The property 'pixelscale' is used to compute the (virtual) focal length: ``f = 1/pixelscale``.  This can be used to convert the image's pixels into meters in
the ROS code: ``1 meter = 1 pixel * 1/f``

Example:

```qml
Image {
    //...

    ImagePublisher {
      id: imgPublisher
      target: parent // object to publish
      topic: "/qmlapp/image" // the desired ROS image topic
      frame: "/map" // reference frame for that image
      width: 800
      height: 600
      latched: false // if you want a latched topic (default: false)
      pixelscale: 1 // inverse of the (virtual) focal length (cf explanation above)
    }

    onPaintedGeometryChanged: imgPublisher.publish() // call publish() everytime the image data changes

}
```

Displaying ROS image topics
---------------------------

This uses a special QML ``ImageProvider`` to read images from a ROS topic. Specify the topic using: `img.source = "image://rosimage/<your topic>"`.

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

TFBroadcaster
-------------

Publishes the position (x,y,theta) of a QML item as a TF frame. Pixel to meters conversion
is controlled by the ``pixelscale`` parameter.

```qml
Item {
  id: test

 TFBroadcaster {
    target: parent // reference to the item whose position is broadcasted
    frame: "test_frame" // name of the TF frame. Mandatory.
    parentframe: "parent_frame" // name of the parent frame. Mandatory.
    pixelscale: 2 // meters = pixel * pixelscale, by default 1
    origin: originItem // reference to a QML item acting as reference point. By default, scene's (0,0)
    zoffset: 1 // if set, used as the `z` coordinate of the TF frame. By default, 0
 }
}
```

TFListener
----------


RosPose
-------

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
