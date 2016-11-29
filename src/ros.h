#ifndef ROS_H
#define ROS_H

#include <thread>

#include<QQuickItem>

/**
 * @brief A ROS bridge for QML
 */
class Ros : public QQuickItem {
Q_OBJECT
    Q_PROPERTY(QQuickItem* target WRITE setTarget MEMBER _target)
    Q_PROPERTY(QString frame WRITE setFrame MEMBER _parentframe)

public:

    Ros(QQuickItem* parent = 0);

    virtual ~Ros();

    void setTarget(QQuickItem* target);
    void setFrame(QString frame);

private:

    QQuickItem* _target;
    QString _parentframe;

    bool _running;
    std::thread _broadcaster_thread;

};

#endif // ROS_H

