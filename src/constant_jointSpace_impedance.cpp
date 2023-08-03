#include <array>
#include <fstream>
#include <unistd.h>
#include <Eigen/Core>
#include <xmate_exception.h>
#include <robot.h>
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "move.h"
#include "ini.h"
#include "torque_from_regression_matrix.h"

using namespace std;
using namespace xmate;

static const double PERIOD = 0.001;

int main() {
    // ============================ INITIALIZATION PART ==========================
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
    static double time = -0.001;

    array<double, 7> q_drag = {{0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0}};
    array<double, 7> q_init{};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, q_drag, robot);
    // ---------------------------------------------------------------------------

    // ============================  PREPARATION PART  ===========================
    array<double, 7> qd_last_array = {{0, 0, 0, 0, 0, 0, 0}};
    Eigen::Map<Eigen::VectorXd> qd_last(qd_last_array.data(), qd_last_array.size());
    array<double, 7> qdd_array = {{0, 0, 0, 0, 0, 0, 0}};
    Eigen::Map<Eigen::VectorXd> qdd(qdd_array.data(), qdd_array.size());

    array<double, 7> q_aim_array = {0, M_PI / 6, 0, M_PI / 3, 0, M_PI / 2, 0};
    Eigen::Map<Eigen::VectorXd> q_aim(q_aim_array.data(), q_aim_array.size());
    array<double, 7> qd_aim_array = {0, 0, 0, 0, 0, 0, 0};
    Eigen::Map<Eigen::VectorXd> qd_aim(qd_aim_array.data(), qd_aim_array.size());
    array<double, 7> qdd_aim_array = {0, 0, 0, 0, 0, 0, 0};
    Eigen::Map<Eigen::VectorXd> qdd_aim(qdd_aim_array.data(), qdd_aim_array.size());

    array<double, 7> torque_array = {};
    Eigen::Map<Eigen::VectorXd> torque(torque_array.data(), torque_array.size());
    array<double, 7> torque_command_array = {};
    Eigen::Map<Eigen::VectorXd> torque_command(torque_command_array.data(), torque_command_array.size());

    Eigen::DiagonalMatrix<double, 7> M_coefficient;
    M_coefficient.diagonal() << 1, 1, 1, 1, 1, 1, 1;
    Eigen::DiagonalMatrix<double, 7> D_coefficient;
    D_coefficient.diagonal() << 1, 1, 1, 1, 1, 1, 1;
    Eigen::DiagonalMatrix<double, 7> K_coefficient;
    K_coefficient.diagonal() << 100, 100, 100, 100, 100, 100, 100;

    cout << "初始化结束，等待回调函数。" << endl;
    // ---------------------------------------------------------------------------

    // ============================ IMPORTANT FUNCTION ===========================
    Torques torque_commit{};

    // 机械臂回调函数
    auto constant_jointSpace_impedance_callback = [&](RCI::robot::RobotState robot_state) -> Torques {
        time += PERIOD;

        Eigen::Map<Eigen::VectorXd> torque(robot_state.tau_m.data(), robot_state.tau_m.size());
        Eigen::Map<Eigen::VectorXd> q(robot_state.q.data(), robot_state.q.size());
        Eigen::Map<Eigen::VectorXd> qd(robot_state.dq_m.data(), robot_state.dq_m.size());

        qdd = (qd - qd_last) / PERIOD;
        qd_last_array = robot_state.dq_m;

        torque_from_regression_matrix(robot_state.q, robot_state.dq_m, qdd_array, torque_array);
        torque_command = torque - (M_coefficient * (qdd_aim - qdd) + D_coefficient * (qd_aim - qd) + K_coefficient * (q_aim - q));
        torque_commit = torque_command_array;

        if (time > 60) {
            cout << "运动结束" << endl;
            return MotionFinished(torque_commit);
        }
        return torque_commit;
    };

    // ---------------------------------------------------------------------------
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);
    robot.Control(constant_jointSpace_impedance_callback);

    return 0;
}