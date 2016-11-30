ROS QML plugin
==============

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
> qmake ..
> make
> make install
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
}
