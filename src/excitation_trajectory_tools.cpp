//
// Created by lab316-test on 7/8/23.
//
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <cmath>
#include "excitation_trajectory_tools.h"

using namespace std;

int main() {
    // 测试函数功能代码
    double t = 0;
    Eigen::MatrixXd Fourier_series_factors(7, 11);
    Eigen::MatrixXd Fourier_series_body(11, 1);
    Eigen::MatrixXd trajectory_real_time_Fourier_5(7, 1);

    string file_path = "/home/aowupang/文档/code/ROKAE_Control_Interface/data/optimal_trajectory/opt_x_initial1.txt";
    get_joint_parameters(file_path, Fourier_series_factors);

    for (int i = 0; i < 100; ++i) {
        t = 0.1 * i;
        get_motion_info(t, trajectory_real_time_Fourier_5, Fourier_series_factors);
        cout << trajectory_real_time_Fourier_5 << endl;
    }

    return 0;
}

int get_joint_parameters(const string &file_path, Eigen::MatrixXd &container) {
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

int get_motion_info(const double &t, Eigen::MatrixXd &container, const Eigen::MatrixXd &Fourier_factors) {
    Eigen::MatrixXd Fourier_series_body(11, 1);

    for (int i = 0; i < 10; i += 2) {
        Fourier_series_body(i, 0) = 1 / (OMEGA * (i + 1)) * sin(OMEGA * (i + 1) * t);
        Fourier_series_body(i, 0) = 1 / (OMEGA * (i + 1)) * cos(OMEGA * (i + 1) * t);
    }

    Fourier_series_body(10, 0) = 1;

    container = Fourier_factors * Fourier_series_body;

    return 0;
}
