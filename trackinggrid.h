#ifndef TRACKINGGRID_H
#define TRACKINGGRID_H

#include <QObject>
#include <vector>
#include<opencv2/opencv.hpp>

class TrackingGrid : public QObject
{
    Q_OBJECT
public:
    explicit TrackingGrid(QObject *parent = nullptr);
    TrackingGrid(int);
    std::vector<int> getPosCornes(std::vector<std::vector<cv::Point2f> >);

signals:

public slots:

private:
    int nPoints;
};

#endif // TRACKINGGRID_H
