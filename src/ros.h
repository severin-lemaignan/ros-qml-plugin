#ifndef ROS_H
#define ROS_H

#include <thread>
#include<QQuickItem>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * @brief A ROS bridge for QML
 */
class RosPositionController : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(bool position MEMBER _position NOTIFY onPositionChanged)
    Q_PROPERTY(QQuickItem* origin MEMBER _origin)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)

public:

    RosPositionController(QQuickItem* parent = 0);

    virtual ~RosPositionController() {}

    void onIncomingPose(const geometry_msgs::PoseStamped&);

private slots:
    void updatePos(double x, double y);

signals:
    void onPositionChanged();

    void onMsgReceived(double x, double y);

private:

    QQuickItem* _origin;
    double _pixel2meter;

    bool _position; // not really used, but required tfor 'onPositionChanged' to be valid in QML

    ros::NodeHandle _node;
    ros::Subscriber _incoming_poses;

};

/**
 * @brief A QML Item that broadcast its target's pose to TF
 */
class TFBroadcaster : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(bool active MEMBER _active)
    Q_PROPERTY(QQuickItem* target WRITE setTarget MEMBER _target)
    Q_PROPERTY(QQuickItem* origin MEMBER _origin)
    Q_PROPERTY(QString parentframe WRITE setParentFrame MEMBER _parentframe)
    Q_PROPERTY(QString frame WRITE setFrame MEMBER _frame)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)

public:

    TFBroadcaster(QQuickItem* parent = 0);

    virtual ~TFBroadcaster();

    void setTarget(QQuickItem* target);
    void setFrame(QString frame);
    void setParentFrame(QString frame);

private:

    void tfPublisher();

    bool _active;
    bool _running;
    bool _initialized;
    std::thread _broadcaster_thread;

    QQuickItem* _target;
    QQuickItem* _origin;
    QString _frame;
    QString _parentframe;

    qreal _pixel2meter;

    ros::NodeHandle _node;
    tf::TransformBroadcaster _br;

};

/**
 * @brief The FootprintsPublisher class provides a QML object that publishes on a
 * *latched topic* (/footprints) the 2D bounding boxes of each of its 'targets'.
 * QML items in 'targets' must have a property 'boundingbox' that contains a Box2D fixture.
 */
class FootprintsPublisher : public QQuickItem {

 Q_OBJECT
    Q_PROPERTY(QVariantList targets WRITE setTargets MEMBER _targets)
    Q_PROPERTY(double pixelscale MEMBER _pixel2meter)

public:

    static const QString topic;

    FootprintsPublisher(QQuickItem* parent = 0);

    virtual ~FootprintsPublisher() {}

    void setTargets(QVariantList targets);

private:

    QVariantList _targets;

    qreal _pixel2meter;

    ros::NodeHandle _node;
    ros::Publisher _publisher;

};

#endif // ROS_H

