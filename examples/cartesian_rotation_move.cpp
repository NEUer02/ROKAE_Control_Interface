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

#include <Eigen/Dense>

#include "ini.h"
#include "print_rci.h"
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

    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);
    
    robot.setCoor({{1,0,0,0,
                    0,1,0,0.1,
                    0,0,1,0.2,
                    0,0,0,1}});

    Eigen::Affine3d initial_transform;
    Eigen::Affine3d rot_change;
    Eigen::Affine3d cur_transform;

    double time = 0;
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kCartesianPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kCartesianPosition);

    static bool init=true;
    std::array<double, 16> init_position;

    CartesianControl cartesian_position_callback;
    cartesian_position_callback = [&](RCI::robot::RobotState robot_state) -> CartesianPose {
        time += 0.001; 
        if(init==true){
            init_position = robot_state.toolTobase_pos_m;
            initial_transform = Eigen::Matrix4d::Map(init_position.data()).transpose();
            init=false;
        }
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }
        constexpr double kRadius = 0.2;
        double angle = M_PI / 4 * (1 - std::cos(M_PI / 5 * time));
        double delta_x = kRadius * std::sin(angle);
        double delta_z = kRadius * (std::cos(angle) - 1);

        rot_change.linear() << Eigen::AngleAxisd(delta_z,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
        Eigen::AngleAxisd(delta_z,Eigen::Vector3d::UnitY()).toRotationMatrix()*
        Eigen::AngleAxisd(delta_z,Eigen::Vector3d::UnitX()).toRotationMatrix();

        cur_transform.linear()<<initial_transform.linear()*rot_change.linear();
        cur_transform.translation() = initial_transform.translation();
        std::array<double, 16> new_pose{};
        Eigen::Map<Eigen::Matrix4d>(&new_pose[0], 4, 4) = cur_transform.matrix().transpose();

        CartesianPose output{};
        output.toolTobase_pos_c = new_pose;

        if(time>20){
            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
        }
        return output;        
    };

    robot.Control(cartesian_position_callback);

    return 0;
}
