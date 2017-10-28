#include "path_planning/path_smoothing.h"
#define MAXD 2
bool PathSmooth::Evaluate(const double *const parameters, double *cost, double *gradient) const {

    double (*X)[2];
    Point *path;
    int count = 0;
    for(int i=0;i<sizeof(*parameters);i=i+2)
    {
        (*(X+count))[0] = parameters[i];
        (*(X+count))[1] = parameters[i+1];
        (path+count)->x = parameters[i];
        (path+count)->y = parameters[i+1];
        count ++;
    }
    cost[0] = 0;
//claculate the objective function
    vector<Point> deltaX;
    vector<Point> obstacle;
    vector<double> deltaTheta;
    vector<double> k;
    Point zero;
    zero.x=0;
    zero.y=0;
    deltaX.push_back(zero);
    deltaTheta.push_back(0.0);
    k.push_back(0.0);
    for(int i=0;i<count;i++) // i = 0 ~ count-1
    {
        double w1 = 10, w2 = 1, f1 = 0, f2 = 0, f3 = 0;
        Point deltaX_i ,deltaX_ii;
        if(i>0) {
            deltaX_i = *(path + i) - *(path + i - 1);
            deltaX.push_back(deltaX_i);
        }
        if(i>0 && i<(count-1)) //i = 1~count-2
        {
            deltaX_ii = *(path+i+1) - *(path+i);
            f1 = w1*pow(value(deltaX_i-deltaX_ii),2);

            double deltaTheta_i = acos(dot(deltaX_i,deltaX_ii)/value(deltaX_i)/value(deltaX_ii));
            deltaTheta.push_back(deltaTheta_i);
            double k_i = deltaTheta_i/value(deltaX_i);
            k.push_back(k_i);
            f2 = w2*pow(k_i,2);
        }

        f3 = 99999999.0;
        vector< vector<geometry_msgs::Point> > obs = getObstacles();
        if(checkIfInsideBoundary(*(path+i)) && checkIfOutsideObstacles(obs,*(path+i)))
        {
            Point o = getClosestObstaclePoint(obs,*(path+i));
            obstacle.push_back(o);
            double minDis = value(*(path+i) - o);
            f3 = pow(minDis - MAXD,2);
        } else{
            Point o;
            o.x = 9999;
            o.y = 9999;
            obstacle.push_back(o);
        }
        cost[0] += f1 + f2 + f3;
    }
//calculate gradient
    if (gradient != NULL) {
        for(int i=0;i<count;i++) // i = 0~count-1
        {
            Point g1, g2, g3;
            g1 = zero;
            g2 = zero;
            g3 = zero;
            if(i>0 && i<(count - 1)) //i = 1~count-2
            {
                Point p1i, p2i, p1i_, p1ii;
                p1i = crossVector(-deltaX[i],deltaX[i+1]);
                p2i = crossVector(deltaX[i+1],-deltaX[i]);
                g1 = - 4*(deltaX[i+1] - deltaX[i]);
                double k1i ,k2i;
                k1i = -1/abs(sin(deltaTheta[i]))/value(deltaX[i]);
                k2i = -deltaTheta[i]/pow(value(deltaX[i]),3);
                g2 = k1i*(p1i+p2i) + k2i*deltaX[i];

                if(i>1 && i<count-2)
                {
                    g1 = 2*(deltaX[i]-deltaX[i-1]) + g1 + 2*(deltaX[i+2]-deltaX[i+1]);

                    p1i_ = crossVector(deltaX[i-1],deltaX[i]);
                    g2 = g2 + -1/abs(sin(deltaTheta[i-1]))/value(deltaX[i-1])*p1i_;

                    p1ii = crossVector(-deltaX[i+2],deltaX[i+1]);
                    double k1ii = -1/abs(sin(deltaTheta[i+1]))/value(deltaX[i+1]);
                    double k2ii = deltaTheta[i+1]/pow(value(deltaX[i+1]),3);
                    g2 = g2 + k1ii*p1ii + k2ii*deltaX[i+1];

                }
                else if(i == count-2)
                {
                    g1 = 2*(deltaX[i]-deltaX[i-1]) + g1 ;
                    p1i_ = crossVector(deltaX[i-1],deltaX[i]);
                    g2 = g2 + -1/abs(sin(deltaTheta[i-1]))/value(deltaX[i-1])*p1i_;
                }
                else if(i == 1)
                {
                    g1 = g1 + 2*(deltaX[i+2]-deltaX[i+1]);

                    p1ii = crossVector(-deltaX[i+2],deltaX[i+1]);
                    double k1ii = -1/abs(sin(deltaTheta[i+1]))/value(deltaX[i+1]);
                    double k2ii = deltaTheta[i+1]/pow(value(deltaX[i+1]),3);
                    g2 = g2 + k1ii*p1ii + k2ii*deltaX[i+1];
                }
            }
            if(obstacle[i].x > 100)
            {
                g3 = zero;
            } else{
                g3 = 2*(value((*(path+i))-obstacle[i])-MAXD)/value((*(path+i))-obstacle[i])*((*(path+i))-obstacle[i]);
            }
            Point g = g1 + g2 + g3;
            gradient[2*i] = g.x;
            gradient[2*i+1] = g.y;
        }
    }
    return true;
}
double PathSmooth::value(Point X) const {
    double val = sqrt(X.x*X.x+X.y*X.y);
    return val;
}
double PathSmooth::dot(Point X, Point Y) const {
    double val = X.x*Y.x + X.y*Y.y;
    return val;
}
bool PathSmooth::checkIfInsideBoundary(Point tempNode) const {
    if(tempNode.x <= 0 || tempNode.y <= 0  || tempNode.x >= 100 || tempNode.y >= 100 ) return false;
    else return true;
}
bool PathSmooth::checkIfOutsideObstacles(vector<vector<geometry_msgs::Point> > &obstArray, Point tempNode) const {
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        AB = (pow(obstArray[i][0].x - obstArray[i][1].x,2) + pow(obstArray[i][0].y - obstArray[i][1].y,2));
        AD = (pow(obstArray[i][0].x - obstArray[i][3].x,2) + pow(obstArray[i][0].y - obstArray[i][3].y,2));
        AMAB = (((tempNode.x - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + (( tempNode.y - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
        AMAD = (((tempNode.x - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + (( tempNode.y - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));
        //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return false;
        }
    }
    return true;
}
vector<vector<geometry_msgs::Point>> PathSmooth::getObstacles() const {
    obstacles obst;
    return obst.getObstacleArray();
}
 Point operator-(Point a, Point b) const {
    Point result;
     result.x = a.x - b.x;
     result.y = a.y - b.y;
     return result;
}
Point operator+(Point a, Point b) const {
    Point result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    return result;
}
friend Point operator-(Point a) const
{
    Point result;
    result.x = -a.x;
    result.y = -a.y;
    return result;
}
Point operator*(double val,Point a) const{
    Point result;
    result.x = val * a.x;
    result.y = val * a.y;
    return result;
}
double PathSmooth::getPoint2LineDis(geometry_msgs::Point a, geometry_msgs::Point b, Point target) const {
    double A=0 , B=0 , C=0;
    if(abs(a.x - b.x) < 0.01)
    {
        A = 1;
        C = -a.x;
    } else{
        double k = (a.y-b.y)/(a.x-b.x);
        double b = a.y - k*a.x;

        A = k;
        B = -1;
        C = b;
    }
    double dist = abs(A*target.x + B*target.y + C)/sqrt(A*A + B*B);
    return dist;
}
Point PathSmooth::getClosetPointOnObstacleLine(geometry_msgs::Point a, geometry_msgs::Point b, Point target) const {
    Point o;
    if(abs(a.x-b.x)<0.01)
    {
        o.x = a.x;
        o.y = target.y;
    }
    else if(abs(a.y-b.y)<0.01)
    {
        o.y = a.y;
        o.x = target.x;
    } else{
        double k = (a.y-b.y)/(a.x-b.x);
        double b = a.y - k*a.x;
        o.x = -(b-target.y-target.x/k)/(k+1/k);
        o.y = k*o.x + b;
    }
    return o;
}
Point PathSmooth::getClosestObstaclePoint(vector<vector<geometry_msgs::Point> > &obstArray, Point tempNode) const {
    double minDis;
    int index_min = -1;
    Point o;
    minDis = (MIN(tempNode.x,tempNode.y));
    for(int i=0; i<obstArray.size(); i++)
    {
        double tempDis = getPoint2LineDis(obstArray[i][0],obstArray[i][1],tempNode);
        if(tempDis < minDis)
        {
            minDis = tempDis;
            index_min = i*1000+0;
        }
        tempDis = getPoint2LineDis(obstArray[i][1],obstArray[i][2],tempNode);
        if(tempDis < minDis)
        {
            minDis = tempDis;
            index_min = i*1000 + 1;
        }
        tempDis = getPoint2LineDis(obstArray[i][2],obstArray[i][3],tempNode);
        if(tempDis < minDis)
        {
            minDis = tempDis;
            index_min = i*1000 + 2;
        }
        tempDis = getPoint2LineDis(obstArray[i][3],obstArray[i][4],tempNode);
        if(tempDis < minDis)
        {
            minDis = tempDis;
            index_min = i*1000 + 3;
        }
    }
    if(index_min < 0)
    {
        if(abs(o.x-minDis) < 0.01)
        {
            o.x = tempNode.x;
            o.y = 0;
        } else{
            o.x = 0;
            o.y = tempNode.y;
        }
        return o;
    }
    int i,j;
    if(index_min < 1000)
    {
        i = 0;
        j = index_min;
    } else
    {
        j = index_min % 1000;
        i = (index_min-j)/1000;
    }
    o = getClosetPointOnObstacleLine(obstArray[i][j],obstArray[i][j+1],tempNode);
    return o;
}
Point PathSmooth::crossVector(Point a, Point b) const{
    Point result;
    result = a - (dot(a,b)/pow(value(b),2)*b);
    a = 1/value(a)/value(b)*a;
    return result;
}