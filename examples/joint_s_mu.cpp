/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 */

/**
 * 关节空间位置运动示例
 */

#include <cmath>
#include <iostream>
#include <functional>

#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.3.41";
    std::string name = "callback";
    uint16_t port = 1337;
    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    // RCI连接机器人
    xmate::Robot robot(ipaddr, port,XmateType::XMATE7_PRO);
    //防止网络连接失败
    sleep(1);

    robot.setMotorPower(1);

    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);

    std::array<double, 7> q_end1 = {{0, 0, 0, 0.4, 0, 0, 0}};
    std::array<double, 7> q_end2 = {{0.1, 0.5, 0.7, 0.6, 0.4, 0.4, 0.9}};
    std::array<double, 7> q_end3 = {{-0.5, 0.8, 0, 1.2, 0, -0.4, 0.1}};
    std::array<double, 7> q_end4 = {{0.4, 0.2, -0.7, 0.4, 0.8, 0.8, -0.6}};
    std::array<double, 7> q_end5 = {{-1.4, -0.6, 0.3, 1.4, 0.2, 0.2, 0}};
    std::array<double, 7> q_end6 = {{1.5, 0.5, 0.3, 1.4, 0.2, 1.0, 0.7}};
    std::array<double, 7> q_end7 = {{0, 0, 0, 0, 0, 0, 0}};

    std::vector<std::array<double, 7>> joint_motion_vector;
    joint_motion_vector.push_back(q_end1);
    joint_motion_vector.push_back(q_end2);
    joint_motion_vector.push_back(q_end3);
    joint_motion_vector.push_back(q_end4);
    joint_motion_vector.push_back(q_end5);
    joint_motion_vector.push_back(q_end6);
    joint_motion_vector.push_back(q_end7);

    int count = 0;
    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    std::array<double, 7> init_position;
    std::array<double, 7> joint_delta;
    static bool init = true;
    double time = 0;
    JointPositions output;

    std::string joint_callback = "joint_callback";
    JointControl joint_position_callback;
    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        time += 0.001; 
        if(init==true){
            init_position = robot_state.q;
            init=false;
        }
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }
        JointMotionGenerator joint_s(0.4, joint_motion_vector[count]);
        joint_s.calculateSynchronizedValues_joint(init_position);
        //用户规划用的时间

        if (joint_s.calculateDesiredValues_joint(time, joint_delta) == false) {
            output = {{init_position[0] + joint_delta[0], init_position[1] + joint_delta[1],
                                    init_position[2] + joint_delta[2], init_position[3] + joint_delta[3],
                                    init_position[4] + joint_delta[4], init_position[5] + joint_delta[5],
                                    init_position[6] + joint_delta[6]}};
        }else{
            count++;
            init_position = output.q;
            time = 0;
            if (count >= joint_motion_vector.size()) {
                std::cout<<"运动结束"<<std::endl;
                return MotionFinished(output);
            }
        }
        return output;        
    };

    robot.Control(joint_position_callback);

    return 0;
}
