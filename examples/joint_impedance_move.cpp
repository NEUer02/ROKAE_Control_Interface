/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 轴空间阻抗运动（s规划）
 */

#include <cmath>
#include <functional>

#include <iostream>
#include "robot.h"
#include "ini.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "duration.h"
#include "move.h"

#include <unistd.h>

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[])
{
    std::string ipaddr = "192.168.3.41";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file))
    {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    xmate::Robot robot(ipaddr, port,XmateType::XMATE7_PRO);
    sleep(1);

    robot.setMotorPower(1);
    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.1,q_init,q_drag,robot);
    sleep(1);

    robot.setJointImpedance({{1000, 1000, 1000, 1000, 100, 100, 100}});

    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointImpedance, RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    std::array<double, 7> init_position;
    static bool init = true;
    double time = 0;

    JointControl joint_position_callback;
    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        
        if(init==true){
            init_position = robot_state.q;
            init=false;
        }
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }

        double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

        JointPositions output = {{init_position[0] + delta_angle, init_position[1] + delta_angle,
                                            init_position[2] + delta_angle, init_position[3] - delta_angle,
                                            init_position[4] + delta_angle, init_position[5] - delta_angle,
                                            init_position[6] + delta_angle}}; 
        time += 0.001; 
        if(time>20){
            std::cout<<"运动结束："<<std::endl;
            return MotionFinished(output);
        }
        return output;        
    };

    robot.Control(joint_position_callback);
    return 0;
}
