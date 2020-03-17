#ifndef EVAL_CONTINUITY
#define EVAL_CONTINUITY
#include <iostream>
#include <Eigen/Eigen>
#include <kdl/jacobian.hpp>
#include <cmath>

double eval_continuity(KDL::Jacobian&, KDL::Jacobian&);

#endif 