//
// Created by yt on 10/31/17.
//

#include <path_planning/cg_solver.h>


CG_Solver::CG_Solver(visualization_msgs::Marker &path) {
    numParam = path.points.size();

//    initX << path.points[0].x , path.points[0].y;
//    goalX << path.points[numParam+1].x , path.points[numParam+1].y;
    Eigen::MatrixX2d temp(numParam,2);
    Xi = temp;
    for(int i=0;i<numParam;i++)
    {
        Eigen::RowVector2d p;
        p << path.points[i].x , path.points[i].y;
        Xi.row(i) = p;
    }
}

bool CG_Solver::checkIfInsideBoundary(Eigen::RowVector2d tempNode) {
    if(tempNode[0] <= 0 || tempNode[1] <= 0  || tempNode[0] >= 100 || tempNode[1] >= 100 ) return false;
    else return true;
}
bool CG_Solver::checkIfOutsideObstacles(vector<vector<geometry_msgs::Point> > &obstArray, Eigen::RowVector2d tempNode) {
    double AB, AD, AMAB, AMAD;

    for(int i=0; i<obstArray.size(); i++)
    {
        AB = (pow(obstArray[i][0].x - obstArray[i][1].x,2) + pow(obstArray[i][0].y - obstArray[i][1].y,2));
        AD = (pow(obstArray[i][0].x - obstArray[i][3].x,2) + pow(obstArray[i][0].y - obstArray[i][3].y,2));
        AMAB = (((tempNode[0] - obstArray[i][0].x) * (obstArray[i][1].x - obstArray[i][0].x)) + (( tempNode[1] - obstArray[i][0].y) * (obstArray[i][1].y - obstArray[i][0].y)));
        AMAD = (((tempNode[0] - obstArray[i][0].x) * (obstArray[i][3].x - obstArray[i][0].x)) + (( tempNode[1] - obstArray[i][0].y) * (obstArray[i][3].y - obstArray[i][0].y)));
        //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        if((0 < AMAB) && (AMAB < AB) && (0 < AMAD) && (AMAD < AD))
        {
            return false;
        }
    }
    return true;
}
vector< vector<geometry_msgs::Point> > CG_Solver::getObstacles() {
    obstacles obst;
    return obst.getObstacleArray();
}
double CG_Solver::getPoint2LineDis(geometry_msgs::Point a, geometry_msgs::Point b, Eigen::RowVector2d target){
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
    double dist = abs(A*target[0] + B*target[1] + C)/sqrt(A*A + B*B);
    return dist;
}
Eigen::RowVector2d CG_Solver::getClosetPointOnObstacleLine(geometry_msgs::Point a, geometry_msgs::Point b, Eigen::RowVector2d target){
    Eigen::RowVector2d o;
    if(abs(a.x-b.x)<0.01)
    {
        o << a.x,target[1];
    }
    else if(abs(a.y-b.y)<0.01)
    {
        o << target[0],a.y;
    } else{
        double k = (a.y-b.y)/(a.x-b.x);
        double b = a.y - k*a.x;
        double x,y;
        x = -(b-target[1]-target[0]/k)/(k+1/k);
        y = k*x + b;
        o << x,y;
    }
    return o;
}
Eigen::RowVector2d CG_Solver::getClosestObstaclePoint(vector<vector<geometry_msgs::Point> > &obstArray, Eigen::RowVector2d tempNode){
    double minDis;
    int index_min = -1;
    Eigen::RowVector2d o;
    minDis = (MIN(abs(tempNode[0]),abs(tempNode[1])));
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
    if(index_min < 0) // more closer to boundary
    {
        if(abs(abs(tempNode[0])-minDis) < 0.01)
        {
            o << tempNode[0],0;
        } else{
            o << 0,tempNode[1];
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

Eigen::RowVector2d CG_Solver::crossVector(Eigen::RowVector2d a, Eigen::RowVector2d b) {
    Eigen::RowVector2d result;
    result = a - a*b.transpose()/pow(b.norm(),2)*b;
    result = result / a.norm()/b.norm();
    return result;
}
///calculate gradient
Eigen::MatrixX2d CG_Solver::getGradient(Eigen::MatrixX2d X, Eigen::MatrixX2d deltaX,
                                        double *deltaTheta) {
    Eigen::MatrixX2d G(this->numParam,2);
    for(int i=0;i<this->numParam;i++){
        if(i == 0){
            Eigen::RowVector2d g1,g2;
            g1 = 2*this->w1*(deltaX.row(i+2)-deltaX.row(i+1));
            double k1ii = -1/abs(sin(deltaTheta[i+1]));
            double k2ii = deltaTheta[i+1]/pow(deltaX.row(i+1).norm(),2);
            Eigen::RowVector2d R2ii = crossVector(-deltaX.row(i+2),deltaX.row(i+1));
            g2 = 2*this->w2*k2ii*(k1ii*R2ii+k2ii*deltaX.row(i+1));
            G.row(i) = g1 + g2;
        }
        else if(i == 1){
            Eigen::RowVector2d g1,g2;
            g1 = this->w1*(-4*(deltaX.row(i+1)-deltaX.row(i)) + 2*(deltaX.row(i+2)-deltaX.row(i+1)));
            double k1i = -1/abs(sin(deltaTheta[i]));
            double k2i = deltaTheta[i]/pow(deltaX.row(i).norm(),2);
            Eigen::RowVector2d R1i ,R2i;
            R1i = this->crossVector(deltaX.row(i),deltaX.row(i+1));
            R2i = this->crossVector(deltaX.row(i+1),deltaX.row(i));
            g2 = 2*this->w2*k2i*(k1i*(R2i-R1i)+k2i*deltaX.row(i));

            double k1ii = -1/abs(sin(deltaTheta[i+1]));
            double k2ii = deltaTheta[i+1]/pow(deltaX.row(i+1).norm(),2);
            Eigen::RowVector2d R2ii = crossVector(-deltaX.row(i+2),deltaX.row(i+1));
            g2 += 2*this->w2*k2ii*(k1ii*R2ii+k2ii*deltaX.row(i+1));
            G.row(i) = g1+g2;
        }
        else if(i == this->numParam-2){
            Eigen::RowVector2d g1,g2;
            g1 = this->w1*(2*(deltaX.row(i)-deltaX.row(i-1)) - 4*(deltaX.row(i+1)-deltaX.row(i)));
            double k1i_ = -2*deltaTheta[i-1]/abs(sin(deltaTheta[i-1]))/pow(deltaX.row(i-1).norm(),2);
            Eigen::RowVector2d R1i_ = crossVector(deltaX.row(i-1),deltaX.row(i));
            g2 = this->w2*k1i_*R1i_;

            double k1i = -1/abs(sin(deltaTheta[i]));
            double k2i = deltaTheta[i]/pow(deltaX.row(i).norm(),2);
            Eigen::RowVector2d R1i ,R2i;
            R1i = this->crossVector(deltaX.row(i),deltaX.row(i+1));
            R2i = this->crossVector(deltaX.row(i+1),deltaX.row(i));
            g2 += 2*this->w2*k2i*(k1i*(R2i-R1i)+k2i*deltaX.row(i));
            G.row(i) = g1+g2;
        }
        else if(i == this->numParam-1){
            Eigen::RowVector2d g1,g2;
            g1 = this->w1*(2*(deltaX.row(i)-deltaX.row(i-1)));
            double k1i_ = -2*deltaTheta[i-1]/abs(sin(deltaTheta[i-1]))/pow(deltaX.row(i-1).norm(),2);
            Eigen::RowVector2d R1i_ = crossVector(deltaX.row(i-1),deltaX.row(i));
            g2 = this->w2*k1i_*R1i_;
            G.row(i) = g1+g2;
        }
        else{
            Eigen::RowVector2d g1,g2;
            g1 = this->w1*(2*(deltaX.row(i)-deltaX.row(i-1)) - 4*(deltaX.row(i+1)-deltaX.row(i)) + 2*(deltaX.row(i+2)-deltaX.row(i+1)));
            double k1i_ = -2*deltaTheta[i-1]/abs(sin(deltaTheta[i-1]))/pow(deltaX.row(i-1).norm(),2);
            Eigen::RowVector2d R1i_ = crossVector(deltaX.row(i-1),deltaX.row(i));
            g2 = this->w2*k1i_*R1i_;

            double k1i = -1/abs(sin(deltaTheta[i]));
            double k2i = deltaTheta[i]/pow(deltaX.row(i).norm(),2);
            Eigen::RowVector2d R1i ,R2i;
            R1i = this->crossVector(deltaX.row(i),deltaX.row(i+1));
            R2i = this->crossVector(deltaX.row(i+1),deltaX.row(i));
            g2 += 2*this->w2*k2i*(k1i*(R2i-R1i)+k2i*deltaX.row(i));

            double k1ii = -1/abs(sin(deltaTheta[i+1]));
            double k2ii = deltaTheta[i+1]/pow(deltaX.row(i+1).norm(),2);
            Eigen::RowVector2d R2ii = crossVector(-deltaX.row(i+2),deltaX.row(i+1));
            g2 += 2*this->w2*k2ii*(k1ii*R2ii+k2ii*deltaX.row(i+1));

            G.row(i) = g1 + g2;
        }
    }
    return G;
}
///calculate func value
double CG_Solver::getFuncValue(Eigen::MatrixX2d &X, Eigen::MatrixX2d &deltaX, double *deltaTheta) {
    if(X.rows() != this->numParam || deltaX.rows() != this->numParam){
        std::cout<<"number of variable is not correct!"<<std::endl;
        return -1;
    }
    deltaX.row(0) << 0,0;
    deltaTheta[0] = 0.0;
    for(int i=1;i<numParam;i++){
        Eigen::RowVector2d temp;
        deltaX.row(i) = X.row(i) - X.row(i-1);
        if(i > 1){
            double dot = deltaX.row(i-1)*deltaX.row(i).transpose();
            deltaTheta[i-1] = acos(dot/deltaX.row(i-1).norm()/deltaX.row(i).norm());
        }
    }
    double f1 = 0;
    double f2 = 0;
    for(int i=1;i<numParam-1;i++){
        Eigen::RowVector2d delta = deltaX.row(i+1) - deltaX.row(i);
        f1 += w1*pow(delta.norm(),2);
        f2 += w2*pow(deltaTheta[i]/deltaX.row(i).norm(),2);
    }
    return f1+f2;
}
double CG_Solver::getFuncValue(Eigen::MatrixX2d &X) {
    Eigen::MatrixX2d deltaX(this->numParam,2);
    double *deltaTheta = new double(numParam);
    deltaX.row(0) << 0,0;
    deltaTheta[0] = 0.0;
    for(int i=1;i<numParam;i++){
        Eigen::RowVector2d temp;
        deltaX.row(i) = X.row(i) - X.row(i-1);
        if(i > 1){
            double dot = deltaX.row(i-1)*deltaX.row(i).transpose();
            deltaTheta[i-1] = acos(dot/deltaX.row(i-1).norm()/deltaX.row(i).norm());
        }
    }
    double f1 = 0;
    double f2 = 0;
    for(int i=1;i<numParam-1;i++){
        Eigen::RowVector2d delta = deltaX.row(i+1) - deltaX.row(i);
        f1 += w1*pow(delta.norm(),2);
        f2 += w2*pow(deltaTheta[i]/deltaX.row(i).norm(),2);
    }
    delete deltaTheta;
    return f1 + f2;
}
void CG_Solver::Solve() {
    Eigen::MatrixX2d d(numParam,2);
    Eigen::MatrixX2d g(numParam,2);
    Eigen::MatrixX2d g_old(numParam,2);
    Eigen::MatrixX2d deltag(numParam,2);
    Eigen::MatrixX2d deltaX(numParam,2);
    double *deltaTheta = new double(numParam);
    double belta;
    double F ;
    double alpha;
    ///init value
    F = this->getFuncValue(this->Xi,deltaX,deltaTheta);
    g = this->getGradient(this->Xi,deltaX,deltaTheta);
    d = -g;
    g_old = g;
    Eigen::Matrix2d temp;
    temp = g.transpose()*g;

    for(int i =0;i< 200;i++)
    {
        ///get step
        alpha = this->stepLineSearch(Xi,d,F);
        ///update vatiable state
        this->Xi = this->Xi + alpha*d;
        ///update gradient
        g = this->getGradient(this->Xi,deltaX,deltaTheta);
        if(alpha < 0.00001){
            d = -g;
        }
        else {
            ///calculate belta;
            deltag = g - g_old;
            Eigen::MatrixX2d temp(2, 2);
            temp = g * deltag.transpose();
            double num = temp(0, 0) + temp(1, 1);
            temp = g_old * g_old.transpose();
            double den = temp(0, 0) + temp(1, 1);
            belta = num / den;
            ///update direction
            d = -g + belta * d;
        }
        g_old = g;
        F = this->getFuncValue(this->Xi,deltaX,deltaTheta);
    }
    delete deltaTheta;
}


double CG_Solver::stepLineSearch(Eigen::MatrixX2d &X, Eigen::MatrixX2d &d, double F) {
    double s_l = 0;
    double s_h = 1;
    double s1 = s_h - (s_h-s_l) * 0.618;
    double s2 = s_l + (s_h-s_l) * 0.618;
    double s = 0;
    while (s2 - s1 >0.0001)
    {
        Eigen::MatrixX2d X1 = X + s1*d;
        Eigen::MatrixX2d X2 = X + s2*d;
        double F1 = this->getFuncValue(X1);
        double F2 = this->getFuncValue(X2);
        if(F1 < F){
            s = s1;
            return s;
        }
        else if(F2 < F)
        {
            s = s2;
            return s;
        }
        if(F1 < F2)
        {
            s_h = s2;
            s1 = s_h - (s_h - s_l)*0.618;
            s2 = s_l + (s_h - s_l)*0.618;
        }
        else
        {
            s_l = s1;
            s1 = s_h - (s_h - s_l)*0.618;
            s2 = s_l + (s_h - s_l)*0.618;
        }
    };
    return s;
}
