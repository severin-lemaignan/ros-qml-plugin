#include <iostream>
#include "ros.h"


using namespace std;

Ros::Ros(QQuickItem *parent):
    _running(false),
    _target(nullptr)
{

}

Ros::~Ros()
{
    if (_running) {
        _running=false;
        _broadcaster_thread.join();
    }

}

void Ros::setTarget(QQuickItem* target)
{
    //if(_target) disconnect(_target);

   _target = target;
    //connect(_target,)
   if (!_running) {
       cout << "Target set to: " << _target->property("id").toString().toStdString() << endl;
       cout << "Target at: " << _target->x() << ", " << _target->y() << endl;
       _running = true;

       _broadcaster_thread = std::thread(&Ros::tfPublisher, this);

   }

}

void Ros::setFrame(QString frame)
{
        _parentframe = frame;
        cout << "Parent frame set to: " << _parentframe.toStdString() << endl;
}

void Ros::tfPublisher()
{
    while(_running) {
       cout << "Target at: " << _target->x() << ", " << _target->y() << endl;
       sleep(1);
    }

}
