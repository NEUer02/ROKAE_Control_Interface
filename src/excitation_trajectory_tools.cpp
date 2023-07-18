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
        Fourier_series_body(i, 0) = 1 / (OMEGA * (i / 2 + 1)) * sin(OMEGA * (i / 2 + 1) * t);
        Fourier_series_body(i + 1, 0) = -1 / (OMEGA * (i / 2 + 1)) * cos(OMEGA * (i / 2 + 1) * t);
    }

    Fourier_series_body(10, 0) = 1;

    container = Fourier_factors * Fourier_series_body;

    return 0;
}
