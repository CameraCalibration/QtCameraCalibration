#include "trackinggrid.h"
#include "geometria.h"

TrackingGrid::TrackingGrid(QObject *parent) : QObject(parent)
{

}

TrackingGrid::TrackingGrid(int n){
    nPoints = n;
}

std::vector<int> TrackingGrid::getPosCornes(std::vector<std::vector<cv::Point2f> > hull){
    std::vector<int> posCornes;
    double piValue = 3.14159265, maxAngle = 148;
    int nHull = (int)hull[0].size();
    for(int i = 0; i < nHull; i++){
        float angle;
        PointFrz A(hull[0][i].x - hull[0][(i==0?nHull:i)-1].x, hull[0][i].y - hull[0][(i==0?nHull:i)-1].y);
        PointFrz B(hull[0][i].x - hull[0][i==nHull-1?0:i+1].x, hull[0][i].y - hull[0][i==nHull-1?0:i+1].y);
        angle = atan2(cross(A,B),dot(A,B)) * 180 / piValue;
        if(abs(angle) <= maxAngle)
            posCornes.push_back(i);
    }
    return posCornes;
}
