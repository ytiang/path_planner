#ifndef PATH_PLANNING_PATH_SMOOTHING_H
#define PATH_PLANNING_PATH_SMOOTHING_H

#include "obstacles.h"
#include "ceres/ceres.h"
#include <visualization_msgs/Marker.h>
#define MIN(a,b) a<b ? a : b
#define MAXD 10

class Vector2
{
public:
    double x;
    double y;
    friend Vector2 operator-(Vector2 a,Vector2 b) ;
    friend Vector2 operator+(Vector2 a,Vector2 b) ;
    friend Vector2 operator*(double val,Vector2 a) ;
    Vector2 operator-();
};

class PathSmooth : public ceres::FirstOrderFunction {
private:
    int numParam;
public:
    PathSmooth(int param);
    virtual bool Evaluate(const double* parameters,
                          double* cost,
                          double* gradient) const;

    virtual int NumParameters() const ;
    double value(Vector2 X) const ;
    double dot(Vector2 X,Vector2 Y) const ;
    bool checkIfInsideBoundary(Vector2 tempNode) const;
    bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, Vector2 tempNode) const;
    vector< vector<geometry_msgs::Point> > getObstacles() const;
    Vector2 getClosestObstaclePoint(vector< vector<geometry_msgs::Point> > &obstArray, Vector2 tempNode) const;
    Vector2 getClosetPointOnObstacleLine(geometry_msgs::Point a, geometry_msgs::Point b, Vector2 target) const;
    double getPoint2LineDis(geometry_msgs::Point a,geometry_msgs::Point b, Vector2 target) const;
    Vector2 crossVector(Vector2 a, Vector2 b) const;

};


#endif //PATH_PLANNING_PATH_SMOOTHING_H

