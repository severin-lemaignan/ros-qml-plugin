#include <cmath>
#include <iostream>
#include <chrono>
#include "ros.h"

using namespace std;

RosPositionController::RosPositionController(QQuickItem *parent):
    _origin(nullptr),
    _pixel2meter(1),
    _incoming_poses(_node.subscribe("/move_base_simple/goal", 1, &RosPositionController::onIncomingPose, this))
{

    connect(this, SIGNAL(onMsgReceived(double, double)),
            this, SLOT(updatePos(double, double)));

}

void RosPositionController::onIncomingPose(const geometry_msgs::PoseStamped &pose)
{
    emit onMsgReceived(pose.pose.position.x, pose.pose.position.y);
}

void RosPositionController::updatePos(double x, double y)
{
    double px,py;
    if (_origin) {
        px = x / _pixel2meter + _origin->x();
        py = -y / _pixel2meter + _origin->y();
    }
    else {
        px = x / _pixel2meter;
        py = -y / _pixel2meter;
    }

    setX(px);
    setY(py);
    emit onPositionChanged();

}

TFBroadcaster::TFBroadcaster(QQuickItem *parent):
    _initialized(false),
    _running(false),
    _target(nullptr),
    _origin(nullptr),
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

        //cout << "Frame set to: " << _frame.toStdString() << endl;
}


void TFBroadcaster::setParentFrame(QString frame)
{

        _parentframe = frame;
        if (!_frame.isEmpty()) _initialized = true;

        //cout << "Parent frame set to: " << _parentframe.toStdString() << endl;
}

void TFBroadcaster::tfPublisher()
{
    while(_running) {
        if(_initialized) {
            double x,y, theta;
            if (_origin) {
                x = (_target->x() - _origin->x()) * _pixel2meter;
                y = -(_target->y() - _origin->y()) * _pixel2meter;
                theta = -(_target->rotation() - _origin->rotation()) * M_PI/180;
            }
            else {
                x = _target->x() * _pixel2meter;
                y = -_target->y() * _pixel2meter;
                theta = -_target->rotation() * M_PI/180;
            }

            tf::Transform transform;
            transform.setOrigin( tf::Vector3(x, y, 0.0) );

            tf::Quaternion q;
            q.setRPY(0, 0, theta);
            transform.setRotation(q);
            _br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _parentframe.toStdString(), _frame.toStdString()));
        }
       this_thread::sleep_for(chrono::milliseconds(100));
    }

}
