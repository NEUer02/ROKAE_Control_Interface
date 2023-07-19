#include <array>
#include <vector>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <xmate_exception.h>
#include <model.h>
#include <robot.h>

#include "ini.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "move.h"

#include <unistd.h>

/**
 * @笛卡尔空间下的导纳拖动
 */
using namespace xmate;
using namespace std;
using namespace Eigen;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;


void save_pos_to_txt(vector<array<double, 16>> &toolTobase_pos_m_sum) {
    fstream pos_record;
    pos_record.open("/home/ea/桌面/20230401/pos/1.txt", ios::out);   //weizhi
    int m = toolTobase_pos_m_sum.size();
    int n = 16;
    for (int i = 0; i < m; i++) {
        pos_record << toolTobase_pos_m_sum[i][3] << " " << toolTobase_pos_m_sum[i][7] << " "
                   << toolTobase_pos_m_sum[i][11] << "\n";
    }
    pos_record.close();

}

void save_force_to_txt(vector<array<double, 6>> &tau_ext_in_base_sum) {
    fstream force_record;
    force_record.open("/home/ea/桌面/20230401/force/1.txt", ios::out);
    int m = tau_ext_in_base_sum.size();
    int n = 6;
    for (int i = 0; i < m; i++) {
        force_record << tau_ext_in_base_sum[i][1] << " " << tau_ext_in_base_sum[i][2] << " "
                     << tau_ext_in_base_sum[i][3] << "\n";
    }
    force_record.close();
}


int main(int argc, char *argv[]) {
    // Check whether the required arguments were passed
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

    const double PI = 3.14159;
    std::array<double, 7> q_init{};
    std::array<double, 7> q_drag = {{0, PI / 6, 0, PI / 3, 0, PI / 2, 0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, q_drag, robot);

    xmate::XmateModel model(&robot, xmate::XmateType::XMATE3_PRO);

    //robot.reg();
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    const double spring_param{1};
    const double damp_param1{3};
    const double damp_param2{500.0};
    const double mass_param_fe{0.0};
    const double mass_param_ne{100.0};

    std::array<double, 6> last_time_pos{};
    std::array<double, 6> d_last_time_pos{};
    std::array<double, 6> current_pos{};
    std::array<double, 6> d_current_pos{};
    std::vector<std::array<double, 16>> toolTobase_pos_m_sum;
    std::vector<std::array<double, 6>> tau_ext_in_base_sum;
    double ddxe, ddye, ddze;
    double dxe, dye, dze;
    double xe, ye, ze;
    double ddTxe, ddTye, ddTze;
    double dTxe, dTye, dTze;
    double Txe, Tye, Tze;
    double dt = 0.001;

    std::array<double, 16> init_position{};
    Eigen::Matrix<double, 6, 7> jacobian;
    Eigen::Matrix<double, 7, 6> jacobian_pinv;

    JointPositions output{};
    JointControl admittance_control_callback;
    admittance_control_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {

        static double time = 0;
        time += 0.001;

        std::array<double, 6> external_force = robot_state.tau_ext_in_base; //基坐标系下的外部力矩
        // std::array<double, 6> external_force = robot_state.tau_ext_in_stiff; //力控坐标系下的外部力矩
        std::array<double, 42> jacobian_array = model.Jacobian(robot_state.q);

        Eigen::Map<Eigen::Matrix<double, 7, 1>> joint_pos(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7, 6>> jacobian_(jacobian_array.data());
        jacobian = jacobian_.transpose();

        std::array<double, 7> joint_v = robot_state.dq_m;
        cout << joint_v[0] << " " << joint_v[1] << " " << joint_v[2] << " " << joint_v[3] << " " << joint_v[4] << " "
             << joint_v[5] << " " << joint_v[6] << endl;


        if (time == 0) {
            last_time_pos = {0, 0, 0, 0, 0, 0};
            d_last_time_pos = {0, 0, 0, 0, 0, 0};
        } else {
            last_time_pos = current_pos;
            d_last_time_pos = d_current_pos;
        }


        // admittance control

        ddxe = (-external_force[0] - damp_param1 * d_last_time_pos[0] - mass_param_fe * last_time_pos[0]) /
               spring_param;
        dxe = d_last_time_pos[0] + ddxe * dt;
        xe = last_time_pos[0] + dxe * dt;

        ddye = (-external_force[1] - damp_param1 * d_last_time_pos[1] - mass_param_fe * last_time_pos[1]) /
               spring_param;
        dye = d_last_time_pos[1] + ddye * dt;
        ye = last_time_pos[1] + dye * dt;

        ddze = (-external_force[2] - damp_param2 * d_last_time_pos[2] - mass_param_ne * last_time_pos[2]) /
               spring_param;
        dze = d_last_time_pos[2] + ddze * dt;
        ze = last_time_pos[2] + dze * dt;

        ddTxe = (external_force[3] - damp_param2 * d_last_time_pos[3] - mass_param_ne * last_time_pos[3]) /
                spring_param;
        dTxe = d_last_time_pos[3] + ddTxe * dt;
        Txe = last_time_pos[3] + dTxe * dt;

        ddTye = (-external_force[4] - damp_param2 * d_last_time_pos[4] - mass_param_ne * last_time_pos[4]) /
                spring_param;
        dTye = d_last_time_pos[4] + ddTye * dt;
        Tye = last_time_pos[4] + dTye * dt;

        ddTze = (external_force[5] - damp_param2 * d_last_time_pos[5] - mass_param_ne * last_time_pos[5]) /
                spring_param;
        dTze = d_last_time_pos[5] + ddTze * dt;
        Tze = last_time_pos[5] + dTze * dt;

        current_pos = {xe, ye, ze, Txe, Tye, Tze};
        d_current_pos = {dxe, dye, dze, dTxe, dTye, dTze};

        Eigen::Map<const Eigen::Matrix<double, 6, 1>> external_velocity(d_current_pos.data());
        Eigen::VectorXd joint_velocity(7);

        // 因为7自由度机械臂的雅可比矩阵维度是7x6，无法计算逆矩阵
        // 此时使用雅可比矩阵的广义逆解
        Eigen::Matrix<double, 6, 6> jacobian_temp = jacobian * jacobian.transpose();
        jacobian_pinv = jacobian.transpose() * jacobian_temp.inverse();
        joint_velocity = jacobian_pinv * external_velocity;

        // 记录机器人末端的位置和力
        toolTobase_pos_m_sum.push_back(robot_state.toolTobase_pos_m);
        tau_ext_in_base_sum.push_back(robot_state.tau_ext_in_base);

        joint_pos << joint_pos + joint_velocity * dt;

        std::array<double, 7> q_c_array{};
        Eigen::VectorXd::Map(&q_c_array[0], 7) = joint_pos;

        output = q_c_array;

        if (time > 60) {
            std::cout << "运动结束" << std::endl;
            save_pos_to_txt(toolTobase_pos_m_sum);
            save_force_to_txt(tau_ext_in_base_sum);
            return MotionFinished(output);
        }
        return output;
    };

    robot.Control(admittance_control_callback);

    //直接按ctrl_c停止，注意急停开关
    return 0;
}
