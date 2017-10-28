#ifndef PATH_PLANNING_PATH_SMOOTHING_H
#define PATH_PLANNING_PATH_SMOOTHING_H

#include "path_planning/obstacles.h"
#include "ceres/ceres.h"c
#include <visualization_msgs/Marker.h>
typedef struct Point{
    double x;
    double y;
}Point;
#define MIN(a,b) a<b ? a : b
class PathSmooth : public ceres::FirstOrderFunction {
public:
    virtual bool Evaluate(const double* parameters,
                          double* cost,
                          double* gradient) const;

    virtual int NumParameters() const { return 2; }
    double value(Point X) const ;
    double dot(Point X,Point Y) const ;
    bool checkIfInsideBoundary(Point tempNode) const;
    bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, Point tempNode) const;
    vector< vector<geometry_msgs::Point> > getObstacles() const;
    Point getClosestObstaclePoint(vector< vector<geometry_msgs::Point> > &obstArray, Point tempNode) const;
    Point getClosetPointOnObstacleLine(geometry_msgs::Point a, geometry_msgs::Point b, Point target) const;
    double getPoint2LineDis(geometry_msgs::Point a,geometry_msgs::Point b, Point target) const;
    Point crossVector(Point a, Point b) const;

    friend Point operator-(Point a,Point b) const;
    friend Point operator+(Point a,Point b) const;
    friend Point operator-(Point a) const;
    friend Point operator*(double val,Point a) const;

};


#endif //PATH_PLANNING_PATH_SMOOTHING_H

