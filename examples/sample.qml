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

import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.15

import Ros 2.0

Window {
    visible: true
    width: 640
    height: 480
    title: qsTr("ROS2-QML sample app")


    Image {
        id: img
        cache: false

        anchors.fill: parent
        opacity: 0.5
        source: "image://rosimage/image_raw"

        Timer {
            interval: 50
            repeat: true
            running: true
            onTriggered: { img.source = ""; img.source = "image://rosimage/image_raw" }
        }
    }

    Label {
        id: label
        text: "publish a string on /input to change this text"
        anchors.centerIn: parent
    }

    RosStringSubscriber {
        topic: "input"
        onTextChanged: {
            label.text = text;
        }
    }

    RosStringPublisher{
        id: hello_publisher
        topic: "hello"

    }

    Button {
        id: btn

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: label.bottom
        anchors.topMargin: 20

        property int count: 0

        text: "Click me to publish a string on /hello"
        onClicked: {
            count += 1;
            hello_publisher.text = "Hello world - click #" + count;
        }

    }
    Label {
        id: label2

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: btn.bottom
        anchors.topMargin: 20

        text: "publish an empty msg on /signal to toggle the blue square"
    }

    RosSignal {
        topic: "signal"
        onTriggered: semaphore.visible = !semaphore.visible
    }

    Rectangle {
        id: semaphore

        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: label2.bottom
        anchors.topMargin: 20

        width: 10
        height: 10

        visible: false
        color: "blue"
    }

}
