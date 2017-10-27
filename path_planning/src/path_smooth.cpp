#include <iostream>
#include"path_smooth.h"
/*
 * 1 GradientProblemSolver::Options
 *  controls the overall behavior of the solver. We list the various settings and their default values below:
 *      1) line_search_direction_type:
 *          Default: LBFGS
 *          Choices: STEEPEST_DESCENT, NONLINEAR_CONJUGATE_GRADIENT, BFGS, LBFGS
 *      2) line_search_type:
 *          Default: WOLFE
 *          Choices: ARMIJO, WOLFE (strong Wolfe conditions).
 *          Note:
 *              in order for the assumptions underlying the BFGS and LBFGS line search direction algorithms to be guaranteed to be satisifed,
 *              the WOLFE line search should be used.
 *      3) nonlinear_conjugate_gradient_type:
 *          Default: FLETCHER_REEVES
 *          Choices: FLETCHER_REEVES, POLAK_RIBIERE, HESTENES_STIEFEL
 *      4) line_search_interpolation_type
 *          Degree of the polynomial used to approximate the objective function
 *          Default: CUBIC
 *          Choices: BISECTION, QUADRATIC, CUBIC
 *  More settings refer: http://ceres-solver.org/gradient_solver.html#gradientproblemsolver-options
 *
 * */

int main() {

    double parameters[2] = {-1.2, 1.0};

    ceres::GradientProblem problem(new Rosenbrock());

    ceres::GradientProblemSolver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    ceres::GradientProblemSolver::Summary summary;
    ceres::Solve(options, problem, parameters, &summary);

    std::cout << summary.FullReport() << "\n";

    return 0;
}