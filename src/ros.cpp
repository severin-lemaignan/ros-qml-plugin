#include <iostream>
#include <chrono>
#include "ros.h"


using namespace std;

void RosNode::setName(QString name)
{

}

TFBroadcaster::TFBroadcaster(QQuickItem *parent):
    _running(false),
    _target(nullptr)
{

}

TFBroadcaster::~TFBroadcaster()
{
    if (_running) {
        _running=false;
        _broadcaster_thread.join();
    }

}

void TFBroadcaster::setTarget(QQuickItem* target)
{
    //if(_target) disconnect(_target);

    const QMetaObject* metaObject = target->metaObject();
    for(int i = metaObject->propertyOffset(); i < metaObject->propertyCount(); ++i)
        cout << QString::fromLatin1(metaObject->property(i).name()).toStdString() << endl;

   _target = target;
    //connect(_target,)
   if (!_running) {
       _running = true;

       _broadcaster_thread = std::thread(&TFBroadcaster::tfPublisher, this);

   }

}

void TFBroadcaster::setParentFrame(QString frame)
{

        _parentframe = frame;
        cout << "Parent frame set to: " << _parentframe.toStdString() << endl;
}

void TFBroadcaster::tfPublisher()
{
    while(_running) {
       cout << "Target " << _target->property("id").toString().toStdString() << " at " << _target->x() << ", " << _target->y() << endl;
       this_thread::sleep_for(chrono::milliseconds(500));
    }

}
