#include "path_planning/path_smoothing.h"


int PathSmooth::NumParameters() const {
    return numParam;
}
PathSmooth::PathSmooth(int param, double *X, double *Y ) {
    numParam = param;
    init.x = X[0];
    init.y = X[1];
    goal.x = Y[0];
    goal.y = Y[1];
}
bool PathSmooth::Evaluate(const double *const parameters, double *cost, double *gradient) const {

    Vector2 *path = new Vector2[numParam/2 + 2];
    *(path) = init;
    int count = 1;
    for(int i=0;i<numParam/2;i++)
    {
        (path+count)->x = parameters[2*i];
        (path+count)->y = parameters[2*i+1];
        count ++;
    }
    *(path+count) = goal;
    count ++;
    if(count != numParam/2+2)
    {
        std::cout<<"count is "<<count<< " while numParam is "<< numParam <<std::endl;
        return false;
    }
    cost[0] = 0;
//claculate the objective function
    vector<Vector2> deltaX;
    vector<Vector2> obstacle;
    vector<double> deltaTheta;
    Vector2 zero;
    deltaX.push_back(zero);
    deltaTheta.push_back(0.0);
    obstacle.push_back(zero);
    for(int i=1;i<count-1;i++) // i = 1 ~ count-2
    {
        double w1 = 10, w2 = 1, f1 = 0, f2 = 0, f3 = 0;
        Vector2 deltaX_i ,deltaX_ii;
        deltaX_ii = *(path+i+1) - *(path+i);
        deltaX_i = (*(path + i)) - (*(path + i - 1));
        deltaX.push_back(deltaX_i);
        f1 = w1*pow(value(deltaX_i-deltaX_ii),2);

        double deltaTheta_i = acos(dot(deltaX_i,deltaX_ii)/value(deltaX_i)/value(deltaX_ii));
        deltaTheta.push_back(deltaTheta_i);
        double k_i = deltaTheta_i/value(deltaX_i);
        f2 = w2*pow(k_i,2);
        f3 = 99999999.0;
        vector< vector<geometry_msgs::Point> > obs = getObstacles();
        Vector2 o;
        if(checkIfInsideBoundary(*(path+i)) && checkIfOutsideObstacles(obs,*(path+i)))
        {
             o = getClosestObstaclePoint(obs,*(path+i));

        } else{
            o = *(path+i);
        }
        obstacle.push_back(o);
        double minDis = value(*(path+i) - o);
        f3 = pow(minDis - MAXD,2);
        cost[0] += f1 + f2 + f3;

    }
//    std::cout << "cost[0]: " << cost[0] << "  count: " << count << std::endl;
//calculate gradient
    if (gradient != NULL) {
        for(int i=1;i<count-1;i++) // i = 1~count-2
        {
            Vector2 g1, g2, g3;
            g1 = zero;
            g2 = zero;
            g3 = zero;

            Vector2 R1i, R2i, R1i_, R2ii;
            R1i = crossVector(deltaX[i],-deltaX[i+1]);
            R2i = crossVector(-deltaX[i+1],-deltaX[i]);
            g1 = - 4*(deltaX[i+1] - deltaX[i]);

            double k1i ,k2i,ki;
            ki = 2*deltaTheta[i]/pow(value(deltaX[i]),2);
            k1i = -1/abs(sin(deltaTheta[i]));
            k2i = -ki/2;
            g2 = ki*(k1i*(-R1i-R2i) + k2i*deltaX[i]);

            if(i>1 && i<count-2)
            {
                g1 = 2*(deltaX[i]-deltaX[i-1]) + g1 + 2*(deltaX[i+2]-deltaX[i+1]);

                R1i_ = crossVector(deltaX[i-1],-deltaX[i]);
                g2 = g2 + -2*deltaTheta[i-1]/abs(sin(deltaTheta[i-1]))/pow(value(deltaX[i-1]),2)*R1i_;

                R2ii = crossVector(-deltaX[i+2],deltaX[i+1]);
                double kii = 2*deltaTheta[i+1]/pow(value(deltaX[i+1]),2);
                double k1ii = -1/abs(sin(deltaTheta[i+1]));
                double k2ii = kii/2;
                g2 = g2 + kii*(k1ii*R2ii + k2ii*deltaX[i+1]);

            }
            else if(i == count-2)
            {
                g1 = 2*(deltaX[i]-deltaX[i-1]) + g1 ;
                R1i_ = crossVector(deltaX[i-1],-deltaX[i]);
                g2 = g2 + -2*deltaTheta[i-1]/abs(sin(deltaTheta[i-1]))/pow(value(deltaX[i-1]),2)*R1i_;
            }
            else if(i == 1)
            {
                g1 = g1 + 2*(deltaX[i+2]-deltaX[i+1]);

                R2ii = crossVector(-deltaX[i+2],deltaX[i+1]);
                double kii = 2*deltaTheta[i+1]/pow(value(deltaX[i+1]),2);
                double k1ii = -1/abs(sin(deltaTheta[i+1]));
                double k2ii = kii/2;
                g2 = g2 + kii*(k1ii*R2ii + k2ii*deltaX[i+1]);
            }

            g3 = 2*(value((*(path+i))-obstacle[i])-MAXD)/value((*(path+i))-obstacle[i])*((*(path+i))-obstacle[i]);

            Vector2 g = g1 + g2 + g3;
            gradient[2*(i-1)] = g.x;
            gradient[2*(i-1)+1] = g.y;
        }
    }
    delete[] path;
    return true;
}
double PathSmooth::value(Vector2 X) const {
    double val = sqrt(X.x*X.x+X.y*X.y);
    return val;
}
double PathSmooth::dot(Vector2 X, Vector2 Y) const {
    double val = X.x*Y.x + X.y*Y.y;
    return val;
}
bool PathSmooth::checkIfInsideBoundary(Vector2 tempNode) const {
    if(tempNode.x <= 0 || tempNode.y <= 0  || tempNode.x >= 100 || tempNode.y >= 100 ) return false;
    else return true;
}
bool PathSmooth::checkIfOutsideObstacles(vector<vector<geometry_msgs::Point> > &obstArray, Vector2 tempNode) const {
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
vector< vector<geometry_msgs::Point> > PathSmooth::getObstacles() const {
    obstacles obst;
    return obst.getObstacleArray();
}
Vector2 Vector2::operator-() {
    Vector2 result;
    result.x = -this->x;
    result.y = -this->y;
    x = -x;
    y = -y;
    return result;
}
Vector2 operator-(Vector2 a, Vector2 b)  {
    Vector2 result;
     result.x = a.x - b.x;
     result.y = a.y - b.y;
     return result;
}
Vector2 operator+(Vector2 a, Vector2 b)  {
    Vector2 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    return result;
}

Vector2 operator*(double val,Vector2 a) {
    Vector2 result;
    result.x = val * a.x;
    result.y = val * a.y;
    return result;
}
double PathSmooth::getPoint2LineDis(geometry_msgs::Point a, geometry_msgs::Point b, Vector2 target) const {
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
Vector2 PathSmooth::getClosetPointOnObstacleLine(geometry_msgs::Point a, geometry_msgs::Point b, Vector2 target) const {
    Vector2 o;
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
Vector2 PathSmooth::getClosestObstaclePoint(vector<vector<geometry_msgs::Point> > &obstArray, Vector2 tempNode) const {
    double minDis;
    int index_min = -1;
    Vector2 o;
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
Vector2 PathSmooth::crossVector(Vector2 a, Vector2 b) const{
    Vector2 result;
    result = a - (dot(a,b)/pow(value(b),2)*b);
    a = 1/value(a)/value(b)*a;
    return result;
}