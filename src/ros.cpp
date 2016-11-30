#include <cmath>
#include <iostream>
#include <chrono>
#include "ros.h"

using namespace std;

TFBroadcaster::TFBroadcaster(QQuickItem *parent):
    _initialized(false),
    _running(false),
    _target(nullptr),
    _frame(""),
    _parentframe(""),
    _pixel2meter(1)
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

   _target = target;

   if (!_running) {
       _running = true;

       _broadcaster_thread = std::thread(&TFBroadcaster::tfPublisher, this);

   }

}

void TFBroadcaster::setFrame(QString frame)
{

        _frame = frame;
        if (!_parentframe.isEmpty()) _initialized = true;

        cout << "Frame set to: " << _frame.toStdString() << endl;
}


void TFBroadcaster::setParentFrame(QString frame)
{

        _parentframe = frame;
        if (!_frame.isEmpty()) _initialized = true;

        cout << "Parent frame set to: " << _parentframe.toStdString() << endl;
}

void TFBroadcaster::tfPublisher()
{
    while(_running) {
        if(_initialized) {
            cout << "Publishing target " << _frame.toStdString() << " at " << _target->x() << ", " << _target->y() << endl;

            tf::Transform transform;
            transform.setOrigin( tf::Vector3(_target->x() * _pixel2meter,
                                             _target->y() * _pixel2meter,
                                             0.0) );

            tf::Quaternion q;
            q.setRPY(0, 0, _target->rotation()*M_PI/180);
            transform.setRotation(q);
            _br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentframe.toStdString(), _frame.toStdString()));
        }
       this_thread::sleep_for(chrono::milliseconds(50));
    }

}
