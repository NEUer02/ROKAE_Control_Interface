/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 笛卡尔空间阻抗控制示例
 */

#include <cmath>
#include <iostream>
#include <functional>

#include "ini.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"

using namespace xmate;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.3.41";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    xmate::Robot robot(ipaddr, port,XmateType::XMATE7_PRO);
    sleep(1);

    robot.setMotorPower(1);
    robot.setFilterLimit(0,10);
    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3,q_init,q_drag,robot);

    // sleep(5);

    robot.setCartesianImpedance({{1000, 1000, 1000, 100, 100, 100}});
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kCartesianImpedance,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kCartesianPosition);

    std::array<double, 16> init_position;
    static bool init = true;
    double time = 0;

    CartesianControl cartesian_position_callback;
    cartesian_position_callback = [&](RCI::robot::RobotState robot_state) -> CartesianPose {
        time += 0.001; 
        if(init==true){
            init_position = robot_state.toolTobase_pos_m;
            init=false;
        }
        constexpr double kRadius = 0.2;
        double angle = M_PI / 4 * (1 - std::cos(M_PI / 2 * time));
        double delta_x = kRadius * std::sin(angle);
        double delta_z = kRadius * (std::cos(angle) - 1);

        CartesianPose output{};
        output.toolTobase_pos_c = init_position;
        output.toolTobase_pos_c[7]+=delta_z;

        if(time>4){
            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
        }
        return output;        
    };
    robot.Control(cartesian_position_callback);
    while(1){
        sleep(5);
        robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kCartesianImpedance,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kCartesianPosition);
        time = 0 ;
        robot.Control(cartesian_position_callback);
        
    }
        

    

    return 0;
}