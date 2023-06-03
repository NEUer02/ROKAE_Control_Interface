/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <Eigen/Core>
#include <array>

namespace xmate {

std::array<double, 3> combineCenterOfMass(double m_ee, const std::array<double, 3>& F_x_Cee, double m_load,
                                          const std::array<double, 3>& F_x_Cload);

Eigen::Matrix3d skewSymmetricMatrixFromVector(const Eigen::Vector3d& input);

std::array<double, 9> combineInertiaTensor(double m_ee, const std::array<double, 3>& F_x_Cee,
                                           const std::array<double, 9>& I_ee, double m_load,
                                           const std::array<double, 3>& F_x_Cload, const std::array<double, 9>& I_load,
                                           double m_total, const std::array<double, 3>& F_x_Ctotal);
}  // namespace xmate