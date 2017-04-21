#ifndef ROSIMAGEPROVIDER_H
#define ROSIMAGEPROVIDER_H

#include <qquickimageprovider.h>
#include <string>
#include <QString>
#include <QSize>
#include <QImage>

#include <image_transport/image_transport.h>

class RosImageProvider : public QQuickImageProvider
{
public:
    RosImageProvider();

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize);

    ImageType imageType() const override {return QQmlImageProviderBase::Image;}

private:
    std::string _topic;

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    image_transport::ImageTransport _it;
    image_transport::Subscriber _sub;

    QImage _last_image;
};

#endif
