//
// File: torque_from_regression_matrix.h
//
// MATLAB Coder version            : 5.1
// C/C++ source code generated on  : 31-Jul-2023 17:34:56
//
#ifndef TORQUE_FROM_REGRESSION_MATRIX_H
#define TORQUE_FROM_REGRESSION_MATRIX_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include <array>
#include <Eigen/Dense>

using namespace std;

// Function Declarations
extern void torque_from_regression_matrix(const array<double, 7> &q, const array<double, 7> &qd, const array<double, 7> &qdd, array<double, 7> &torque);

#endif

//
// File trailer for torque_from_regression_matrix.h
//
// [EOF]
//
