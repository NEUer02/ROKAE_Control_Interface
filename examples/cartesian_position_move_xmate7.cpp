/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 笛卡尔空间位置运动示例
 */

#include <cmath>
#include <iostream>
#include <functional>

#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"

using namespace xmate;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    xmate::Robot robot(ipaddr, port,XmateType::XMATE3);
    sleep(1);
    robot.setMotorPower(1);

    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,PI/3,0,PI/2,0,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);
    
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kCartesianPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kCartesianPosition);

    uint64_t init_time;
    std::array<double, 16> init_position;
    static bool init = true;
    double init_psi;
    double time = 0;

    CartesianControl cartesian_position_callback;
    cartesian_position_callback = [&](RCI::robot::RobotState robot_state) -> CartesianPose {
        time += 0.001; 
        if(init==true){
            init_position = robot_state.toolTobase_pos_m;
            init_psi = robot_state.psi_m;
            init=false;
        }
        constexpr double kRadius = 0.2;
        double angle = M_PI / 4 * (1 - std::cos(M_PI / 5 * time));
        double delta_x = kRadius * std::sin(angle);
        double delta_z = kRadius * (std::cos(angle) - 1);

        CartesianPose output{};
        output.toolTobase_pos_c = init_position;
        output.toolTobase_pos_c[11]+=delta_z;
        output.psi_c = init_psi;

        return output;        
    };

    robot.Control(cartesian_position_callback);
    return 0;
}
