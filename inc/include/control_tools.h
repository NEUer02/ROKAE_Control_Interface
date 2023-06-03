/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>

/**
 * @file control_tools.h
 */

namespace xmate {

/**
 * 判断一个array是否是一个正确的齐次变换矩阵 行优先
 */
inline bool isHomogeneousTransformation(const std::array<double, 16>& transform) noexcept {
    constexpr double kOrthonormalThreshold = 1e-5;

    if (transform[12] != 0.0 || transform[13] != 0.0 || transform[14] != 0.0 || transform[15] != 1.0) {
        return false;
    }
    for (size_t j = 0; j < 3; ++j) {  // i..column
        if (std::abs(std::sqrt(std::pow(transform[j * 4 + 0], 2) + std::pow(transform[j * 4 + 1], 2) + std::pow(transform[j * 4 + 2], 2)) - 1.0) >
            kOrthonormalThreshold) {
            return false;
        }
    }
    for (size_t i = 0; i < 3; ++i) {  // j..row
        if (std::abs(std::sqrt(std::pow(transform[0 * 4 + i], 2) + std::pow(transform[1 * 4 + i], 2) + std::pow(transform[2 * 4 + i], 2)) - 1.0) >
            kOrthonormalThreshold) {
            return false;
        }
    }
    return true;
}

inline void ToArray(Eigen::Matrix3d& rot_matrix, Eigen::Vector3d& p, std::array<double, 16>& array) {
    array = {{rot_matrix(0, 0), rot_matrix(0, 1), rot_matrix(0, 2), p(0), rot_matrix(1, 0), rot_matrix(1, 1), rot_matrix(1, 2), p(1), rot_matrix(2, 0),
              rot_matrix(2, 1), rot_matrix(2, 2), p(2), 0, 0, 0, 1}};
}

inline void Matrix4dToArray(Eigen::Matrix4d& matrix, std::array<double, 16>& array) {
    array = {
        matrix(0, 0), matrix(0, 1), matrix(0, 2), matrix(0, 3), matrix(1, 0), matrix(1, 1), matrix(1, 2), matrix(1, 3),
        matrix(2, 0), matrix(2, 1), matrix(2, 2), matrix(2, 3), matrix(3, 0), matrix(3, 1), matrix(3, 2), matrix(3, 3),
    };
}

inline void ArrayTo(std::array<double, 16>& array, Eigen::Matrix3d& rot_matrix, Eigen::Vector3d& p) {
    rot_matrix << array[0], array[1], array[2], array[4], array[5], array[6], array[8], array[9], array[10];
    p << array[3], array[7], array[11];
}

inline void ArrayToMatrix4d(Eigen::Matrix4d& matrix, std::array<double, 16>& array) {
    matrix << array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7], array[8], array[9], array[10], array[11], array[12], array[13],
        array[14], array[15];
}

inline void EulerToMatrix(Eigen::Vector3d& euler, Eigen::Matrix3d& matrix) {
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()));
    matrix = yawAngle * pitchAngle * rollAngle;
}

/**
 * Determines whether the current OS kernel is a realtime kernel.
 *
 * On Linux, this checks for the existence of `/sys/kernel/realtime`.
 *
 * @return True if running a realtime kernel, false otherwise.
 */
bool hasRealtimeKernel();

/**
 * Sets the current thread to the highest possible scheduler priority.
 *
 * @param[out] error_message Contains an error message if the scheduler priority
 * cannot be set successfully.
 *
 * @return True if successful, false otherwise.
 */
bool setCurrentThreadToHighestSchedulerPriority(std::string* error_message);

}  // namespace xmate
