// joint_excitation_demo.cpp

// This program demonstrates joint space excitation on a XMATE robot
// using Fourier series trajectory generation.

// It connects to a XMATE robot, reads Fourier series parameters from file,
// generates joint space trajectories, sends position commands, and logs
// joint torque, position and velocity data during motion.

// Authors: Pang Hanyu
// Created: 2023-07-29

#include <array>
#include <fstream>
#include <unistd.h>

// XMATE libraries
#include <Eigen/Dense>
#include <Eigen/Core>
#include <xmate_exception.h>
#include <robot.h>

// headers for trajectory generation
#include "excitation_trajectory_tools.h"

// XMATE control command headers
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"

// XMATE motion control headers
#include "move.h"

// INI file parsing
#include "ini.h"


using namespace xmate;
using namespace std;
using JointControl = function<JointPositions(RCI::robot::RobotState robot_state)>;

const string SAVE_PATH = "../../data/joint_force_log/";
const string FACTORS_PATH = "../../data/optimal_trajectory/opt_x.txt";

inline void record_in_txt(const vector<array<double, 7>> &record);

inline void record_in_csv(const vector<array<double, 7>> &record);

int main() {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    std::string file = "../..xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    xmate::Robot robot(ipaddr, port, XmateType::XMATE3_PRO);
    sleep(1);   // 为保证机械臂运行稳定
    robot.setMotorPower(1);

    Eigen::MatrixXd Fourier_series_factors(7, 11);
    Eigen::MatrixXd Fourier_series_body(11, 1);
    Eigen::MatrixXd trajectory_real_time_Fourier_5(7, 1);
    static double time = -0.001;

    // 计算初始时刻的关节位置
    get_Fourier_parameters(FACTORS_PATH, Fourier_series_factors);
    get_motion_info(0, trajectory_real_time_Fourier_5, Fourier_series_factors);

    array<double, 7> q_drag = {{0, 0, 0, 0, 0, 0, 0}};
    Eigen::Map<Eigen::MatrixXd>(q_drag.data(), trajectory_real_time_Fourier_5.rows(),
                                trajectory_real_time_Fourier_5.cols()) = trajectory_real_time_Fourier_5;

    array<double, 7> q_init{};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, q_drag, robot);

    // ============================ IMPORTANT FUNCTION ===========================
    vector<array<double, 7>> record;
    JointPositions command_point{};

    // 机械臂回调函数
    auto excitation_trajectory_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        array<double, 7> tmp{};

        time += 0.001;

        get_motion_info(time, trajectory_real_time_Fourier_5, Fourier_series_factors);
        Eigen::Map<Eigen::MatrixXd>(tmp.data(), trajectory_real_time_Fourier_5.rows(),
                                    trajectory_real_time_Fourier_5.cols()) = trajectory_real_time_Fourier_5;
        command_point = tmp;

        auto torque_joint = robot_state.tau_m;
        auto point_joint = robot_state.q;
        auto speed_joint = robot_state.dq_m;
        record.push_back(torque_joint);
        record.push_back(point_joint);
        record.push_back(speed_joint);

        if (time > 20) {
            std::cout << "运动结束" << std::endl;
            record_in_txt(record);
            record_in_csv(record);
            return MotionFinished(command_point);
        }
        return command_point;
    };

    // ----------------------------------------------------------------------------
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    robot.Control(excitation_trajectory_callback);

    return 0;
}

inline void record_in_txt(const vector<array<double, 7>> &record) {
    chrono::system_clock::time_point now = chrono::system_clock::now();
    time_t nowTime = chrono::system_clock::to_time_t(now);
    tm *currentTime = localtime(&nowTime);
    ostringstream oss;
    oss << put_time(currentTime, "%Y-%m-%d_%H-%M");

    string filename = oss.str();
    string txt_name = SAVE_PATH + filename + ".txt";

    ofstream record_txt;
    record_txt.open(txt_name, ios::app);

    for (int i = 0; i < record.size(); i = i + 3) {
        record_txt << "frame: " << i / 3;
        record_txt << " torque: ";
        for (auto torque: record[i]) {
            record_txt << torque << " ";
        }
        record_txt << " point: ";
        for (auto point: record[i + 1]) {
            record_txt << point << " ";
        }
        record_txt << " speed: ";
        for (auto speed: record[i + 2]) {
            record_txt << speed << " ";
        }
        record_txt << "\n";
    }

    record_txt.close();
}

inline void record_in_csv(const vector<array<double, 7>> &record) {
    chrono::system_clock::time_point now = chrono::system_clock::now();
    time_t nowTime = chrono::system_clock::to_time_t(now);
    tm *currentTime = localtime(&nowTime);
    ostringstream oss;
    oss << put_time(currentTime, "%Y-%m-%d_%H-%M");

    string filename = oss.str();
    string csv_name = SAVE_PATH + filename + ".csv";

    ofstream record_csv;
    record_csv.open(csv_name, ios::app);

    const vector<string> col_names{"时间",
                                   "力矩1", "力矩2", "力矩3", "力矩4", "力矩5", "力矩6", "力矩7",
                                   "位置1", "位置2", "位置3", "位置4", "位置5", "位置6", "位置7",
                                   "速度1", "速度2", "速度3", "速度4", "速度5", "速度6", "速度7"};
    for (auto &name: col_names) {
        record_csv << name << ",";
    }
    record_csv << "\n";

    for (int i = 0; i < record.size(); i = i + 3) {
        record_csv << i / 3 << ", ";
        for (auto torque: record[i]) {
            record_csv << torque << ", ";
        }
        for (auto point: record[i + 1]) {
            record_csv << point << ", ";
        }
        for (auto speed: record[i + 2]) {
            record_csv << speed << ", ";
        }
        record_csv << "\n";
    }
    record_csv.close();
}
