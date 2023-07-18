#include <array>
#include <fstream>
#include <unistd.h>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <xmate_exception.h>
#include <model.h>
#include <robot.h>
#include <iomanip>

#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "move.h"

#include "excitation_trajectory_tools.h"
#include "ini.h"


using namespace xmate;
using namespace std;
using JointControl = function<JointPositions(RCI::robot::RobotState robot_state)>;

const string SAVE_PATH = "/home/aowupang/文档/code/ROKAE_Control_Interface/data/joint_force_log/";
const string FACTORS_PATH = "/home/aowupang/文档/code/ROKAE_Control_Interface/data/optimal_trajectory/opt_x.txt";

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
    sleep(1);
    robot.setMotorPower(1);

    //------
    Eigen::MatrixXd Fourier_series_factors(7, 11);
    Eigen::MatrixXd Fourier_series_body(11, 1);
    Eigen::MatrixXd trajectory_real_time_Fourier_5(7, 1);
    static double time = -0.001;

    get_joint_parameters(FACTORS_PATH, Fourier_series_factors);
    get_motion_info(0, trajectory_real_time_Fourier_5, Fourier_series_factors);

    array<double, 7> q_drag = {{0, 0, 0, 0, 0, 0, 0}};
    Eigen::Map<Eigen::MatrixXd>(q_drag.data(), trajectory_real_time_Fourier_5.rows(),
                                trajectory_real_time_Fourier_5.cols()) =
            COMPRESSION_RATIO * trajectory_real_time_Fourier_5;

    //------
    array<double, 7> q_init{};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, q_drag, robot);

// ===================================================================================
    vector<array<double, 7>> record;
    JointPositions command_point{};
    auto excitation_trajectory_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        array<double, 7> tmp{};

        time += 0.001;

        get_motion_info(time, trajectory_real_time_Fourier_5, Fourier_series_factors);
        trajectory_real_time_Fourier_5 = COMPRESSION_RATIO * trajectory_real_time_Fourier_5;
        Eigen::Map<Eigen::MatrixXd>(tmp.data(), trajectory_real_time_Fourier_5.rows(),
                                    trajectory_real_time_Fourier_5.cols()) = trajectory_real_time_Fourier_5;
        command_point = tmp;

        auto torque_joint = robot_state.tau_m;
        record.push_back(torque_joint);

        if (time > 20) {
            std::cout << "运动结束" << std::endl;
            record_in_txt(record);
            record_in_csv(record);
            return MotionFinished(command_point);
        }
        return command_point;
    };

// ===================================================================================
    //robot.reg();
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    robot.Control(excitation_trajectory_callback);

    //直接按ctrl_c停止，注意急停开关
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
    for (const auto &row: record) {
        for (const auto &torque: row) {
            record_txt << torque << " ";
        }
        record_txt << std::endl;
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
    for (const auto &row: record) {
        for (const auto &torque: row) {
            record_csv << torque << " ";
        }
        record_csv << std::endl;
    }
    record_csv.close();
}
