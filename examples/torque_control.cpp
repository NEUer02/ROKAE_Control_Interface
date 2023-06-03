/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#include <array>
#include <cmath>
#include <functional>
#include <iostream>

#include <Eigen/Dense>

#include <duration.h>
#include <xmate_exception.h>
#include <model.h>
#include <robot.h>

#include "ini.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "print_rci.h"
#include "move.h"

#include <unistd.h>

/**
 * @直接力矩控制，笛卡尔空间的阻抗控制，运行demo的过程中，请确保手持急停
 */
using namespace xmate;
using TorqueControl = std::function<Torques(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[])
{
    // Check whether the required arguments were passed
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    std::string file = "../..xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file))
    {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    xmate::Robot robot(ipaddr, port,XmateType::XMATE3_PRO);
    sleep(1);
    robot.setMotorPower(1);

    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);

    xmate::XmateModel model(&robot,xmate::XmateType::XMATE3_PRO);

    //robot.reg();
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);

    // Compliance parameters
    const double translational_stiffness{200.0};
    const double rotational_stiffness{5.0};
    Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 0.0 * sqrt(translational_stiffness) *
                                       Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 0.0 * sqrt(rotational_stiffness) *
                                           Eigen::MatrixXd::Identity(3, 3);


    std::array<double, 16> init_position;
    static bool init = true;
    Eigen::Matrix<double, 6, 7> jacobian;

    TorqueControl  torque_control_callback;
    torque_control_callback = [&](RCI::robot::RobotState robot_state) -> Torques {
        
        static double time=0;
        time += 0.001; 
        if(init==true){
            init_position = robot_state.toolTobase_pos_m;
            init=false;
        }
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(init_position.data()).transpose());
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.linear());

        std::array<double, 42> jacobian_array =model.Jacobian(robot_state.q);
        std::array<double, 7> gravity_array = model.GetTau(robot_state.q, robot_state.dq_m, robot_state.ddq_c, xmate::TauType::TAU_GRAVITY);
        std::array<double, 7> friction_array = model.GetTau(robot_state.q, robot_state.dq_m, robot_state.ddq_c, xmate::TauType::TAU_FRICTION);

        // convert to Eigen
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> friction(friction_array.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 6>> jacobian_(jacobian_array.data());
        jacobian = jacobian_.transpose();
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_m(robot_state.dq_m.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.toolTobase_pos_m.data()).transpose());
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

        // compute error to desired equilibrium pose
        // position error
        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << position - position_d;

        // orientation error
        // "difference" quaternion
        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0)
        {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -transform.linear() * error.tail(3);

        // compute control
        Eigen::VectorXd tau_d(7);

        // cartesion space impedance calculate && map to joint space
        tau_d << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq_m));

        std::array<double, 7> tau_d_array{};
        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

        Torques output{};
        output.tau_c = tau_d_array;

        if(time>20){
            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
        }
        return output;        
    };

    // start real-time control loop
    std::cout  << "Make sure you have the user stop at hand!" << std::endl
			   << "After starting try to push the robot and see how it reacts." << std::endl
			   << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque,
    //                 RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);

    robot.Control(torque_control_callback);

    //直接按ctrl_c停止，注意急停开关
    return 0;
}
