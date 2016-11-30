#ifndef ROS_H
#define ROS_H

#include <thread>
#include<QQuickItem>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

/**
 * @brief A ROS bridge for QML
 */
class RosNode : public QQuickItem {
Q_OBJECT
    //Q_PROPERTY(QString pixelscale MEMBER _pixel2meter)
    //Q_PROPERTY(QString node READ setName MEMBER _name)

public:

    RosNode(QQuickItem* parent = 0) {}

    virtual ~RosNode() {}

    void setName(QString name);

private:

    ros::NodeHandle _node;
    qreal _pixel2meter;


};

/**
 * @brief A QML Item that broadcast its target's pose to TF
 */
class TFBroadcaster : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QQuickItem* target WRITE setTarget MEMBER _target)
    Q_PROPERTY(QString parentframe WRITE setParentFrame MEMBER _parentframe)
    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
    Q_PROPERTY(qreal pixelscale MEMBER _pixel2meter)

public:

    TFBroadcaster(QQuickItem* parent = 0);

    virtual ~TFBroadcaster();

    void setTarget(QQuickItem* target);
    void setFrame(QString frame);
    void setParentFrame(QString frame);

private:

    void tfPublisher();

    bool _running;
    bool _initialized;
    std::thread _broadcaster_thread;

    QQuickItem* _target;
    QString _frame;
    QString _parentframe;

    qreal _pixel2meter;

    ros::NodeHandle _node;
    tf::TransformBroadcaster _br;

};

#endif // ROS_H

