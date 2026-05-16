#include "qp_admm.hpp"

namespace cppplot::optimization {

void QPADMMSolver::setup(
        const QPProblem& problem,
        const QPSettings& settings)
{

    problem_ = problem;
    settings_ = settings;

    size_t n = problem.n;

    x_ = Vector::zeros(n);
    z_ = Vector::zeros(problem.m);
    y_ = Vector::zeros(problem.m);

    factorize();
}

void QPADMMSolver::factorize()
{

    double rho = settings_.rho;

    Matrix At = problem_.A.transpose();

    K_ = problem_.H + rho * (At * problem_.A);

}

QPSolution QPADMMSolver::solve()
{

    const Matrix& H = problem_.H;
    const Matrix& A = problem_.A;

    const Vector& f = problem_.f;

    const Vector& l = problem_.l;
    const Vector& u = problem_.u;

    double rho = settings_.rho;

    size_t n = problem_.n;

    QPSolution sol;

    for(int k=0;k<settings_.max_iter;k++)
    {

        // x-update

        Vector rhs =
            -f
            + rho * A.transpose() * (z_ - y_);

        x_ = K_.solve(rhs);

        // z-update

        Vector Ax = A * x_;

        Vector v = Ax + y_;

        z_ = projection(v, l, u);

        // dual update

        y_ = y_ + Ax - z_;

        // convergence check

        double primal =
            (Ax - z_).norm();

        if(primal < settings_.tol)
        {

            sol.converged = true;
            sol.iterations = k;

            break;

        }

    }

    sol.x = x_;
    sol.cost =
        0.5 * (x_.transpose()*H*x_)(0,0)
        + (f.transpose()*x_)(0,0);

    return sol;
}

Vector QPADMMSolver::projection(
        const Vector& v,
        const Vector& l,
        const Vector& u)
{

    Vector z = v;

    for(size_t i=0;i<v.size();i++)
    {

        if(z[i] < l[i])
            z[i] = l[i];

        if(z[i] > u[i])
            z[i] = u[i];

    }

    return z;
}
