//
// Created by aowupang on 23-7-9.
//

#ifndef XMATE_TEST_EXCITATION_TRAJECTORY_TOOLS_H
#define XMATE_TEST_EXCITATION_TRAJECTORY_TOOLS_H

#include "cmath"
#include "Eigen/Dense"

const double OMEGA = 0.1 * M_PI;

int get_joint_parameters(const std::string &file_path, Eigen::MatrixXd &container);

int get_motion_info(const double &t, Eigen::MatrixXd &container, const Eigen::MatrixXd &Fourier_factors);

#endif //XMATE_TEST_EXCITATION_TRAJECTORY_TOOLS_H
