
#ifndef PATH_PLANNING_PATH_SMOOTH_H
#define PATH_PLANNING_PATH_SMOOTH_H
#include "ceres/ceres.h"

class Rosenbrock : public ceres::FirstOrderFunction {
public:
    virtual bool Evaluate(const double* parameters,
                          double* cost,
                          double* gradient) const {
        const double x = parameters[0];
        const double y = parameters[1];

        cost[0] = (1.0 - x) * (1.0 - x) + 100.0 * (y - x * x) * (y - x * x);
        if (gradient != NULL) {
            gradient[0] = -2.0 * (1.0 - x) - 200.0 * (y - x * x) * 2.0 * x;
            gradient[1] = 200.0 * (y - x * x);
        }
        return true;
    }

    virtual int NumParameters() const { return 2; }
};


#endif //PATH_PLANNING_PATH_SMOOTH_H
