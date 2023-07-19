//
// Created by lab316-test on 7/8/23.
//
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <array>

#include <cmath>
#include "excitation_trajectory_tools.h"

using namespace std;

/**
 * @brief 从文件中获取傅里叶级数的参数
 *
 * @param file_path 文件路径
 * @param container 返回的关节参数矩阵
 * @return int 成功返回0,失败返回-1
 */
int get_Fourier_parameters(const string &file_path, Eigen::MatrixXd &container) {
    ifstream file(file_path);
    if (!file) {
        cout << "无法打开文件" << endl;
        return -1;
    }

    string line;
    double parameter;

    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 11; j++) {
            getline(file, line);
            istringstream iss(line);
            if (iss >> parameter) {
                container(i, j) = parameter;
            }
        }
    }

    file.close();
    return 0;
}

/**
 * @brief 通过傅里叶级数的参数和给定的时间计算出7个关节在此时刻的角度
 *
 * @param t 时间
 * @param container 返回的运动信息矩阵
 * @param Fourier_factors Fourier系数矩阵
 * @return int 成功返回0
 */
int get_motion_info(const double &t, Eigen::MatrixXd &container, const Eigen::MatrixXd &Fourier_factors) {
    Eigen::MatrixXd Fourier_series_body(11, 1);

    for (int i = 0; i < 10; i += 2) {
        Fourier_series_body(i, 0) = 1 / (OMEGA * (i / 2 + 1)) * sin(OMEGA * (i / 2 + 1) * t);
        Fourier_series_body(i + 1, 0) = -1 / (OMEGA * (i / 2 + 1)) * cos(OMEGA * (i / 2 + 1) * t);
    }

    Fourier_series_body(10, 0) = 1;

    container = COMPRESSION_RATIO * Fourier_factors * Fourier_series_body;

    return 0;
}
