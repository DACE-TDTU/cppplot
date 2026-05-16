#pragma once

#include "matrix.hpp"

namespace cppplot::optimization {

struct QPSolution {

    Vector x;

    double cost;

    int iterations;

    bool converged;

};

}