//
// Created by aowupang on 23-7-9.
//

#ifndef XMATE_TEST_EXCITATION_TRAJECTORY_TOOLS_H
#define XMATE_TEST_EXCITATION_TRAJECTORY_TOOLS_H

#include "cmath"
#include "Eigen/Dense"

using namespace std;

const double OMEGA = 0.1 * M_PI;
const double COMPRESSION_RATIO = 0.79;

/**
 * @brief 从文件中获取傅里叶级数的参数
 *
 * @param file_path 文件路径
 * @param container 返回的关节参数矩阵
 * @return int 成功返回0,失败返回-1
 */
int get_Fourier_parameters(const string &file_path, Eigen::MatrixXd &container);

/**
 * @brief 通过傅里叶级数的参数和给定的时间计算出7个关节在此时刻的角度
 *
 * @param t 时间
 * @param container 返回的运动信息矩阵
 * @param Fourier_factors Fourier系数矩阵
 * @return int 成功返回0
 */
int get_motion_info(const double &t, Eigen::MatrixXd &container, const Eigen::MatrixXd &Fourier_factors);

#endif //XMATE_TEST_EXCITATION_TRAJECTORY_TOOLS_H
