//
// Created by yt on 10/31/17.
//

#ifndef PATH_PLANNING_CG_SOLVER_H
#define PATH_PLANNING_CG_SOLVER_H

#include <vector>
#include "obstacles.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#define MIN(a,b) a<b ? a : b
#define MAXD 20
class CG_Solver {
private:
    const double w1=1 , w2=1;
    int numParam;
    Eigen::MatrixX2d Xi;
    Eigen::RowVector2d initX , goalX;
public:
    double tolorence;
    CG_Solver(visualization_msgs::Marker &path);
    double getFuncValue(Eigen::MatrixX2d &X, Eigen::MatrixX2d &deltaX, double *deltaTheta);
    double getFuncValue(Eigen::MatrixX2d &X);
    Eigen::MatrixX2d getGradient(Eigen::MatrixX2d X, Eigen::MatrixX2d deltaX, double *deltaTheta);

    bool checkIfInsideBoundary(Eigen::RowVector2d tempNode);
    bool checkIfOutsideObstacles(vector< vector<geometry_msgs::Point> > &obstArray, Eigen::RowVector2d tempNode);
    vector< vector<geometry_msgs::Point> > getObstacles() ;
    Eigen::RowVector2d getClosestObstaclePoint(vector< vector<geometry_msgs::Point> > &obstArray, Eigen::RowVector2d tempNode);
    double getPoint2LineDis(geometry_msgs::Point a,geometry_msgs::Point b, Eigen::RowVector2d target);
    Eigen::RowVector2d getClosetPointOnObstacleLine(geometry_msgs::Point a, geometry_msgs::Point b, Eigen::RowVector2d target) ;
    Eigen::RowVector2d crossVector(Eigen::RowVector2d a , Eigen::RowVector2d b );
    void Solve();
    double stepLineSearch(Eigen::MatrixX2d &X, Eigen::MatrixX2d &d, double F);
};

#endif //PATH_PLANNING_CG_SOLVER_H
