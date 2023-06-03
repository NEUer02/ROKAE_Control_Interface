/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 */

/**
 * 笛卡尔空间位置运动示例
 */

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <functional>

#include "cart_motion.h"
#include "ini.h"
#include "model.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"
#include "model.h"

#include <unistd.h>

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

    bool limit = true;
    robot.setFilterLimit(limit, 10);
    robot.setCollisionBehavior({{15,30,15,20, 8,10, 6 }});
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kCartesianPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kCartesianPosition);

    uint64_t init_time;
    std::array<double, 16> init_position;

    xmate::XmateModel model(&robot,XmateType::XMATE3);
    double delta_s;
    Eigen::Quaterniond rot_cur;
    Eigen::Matrix3d mat_cur;
    std::array<double, 16> pose_end;

    int count = 0;
    double time = 0;
    static bool init = true;

    std::array<double, 16> pose_end1;
    pose_end1.fill(0.0);
    std::array<double, 16> pose_end2;
    pose_end2.fill(0.0);
    std::array<double, 16> pose_end3;
    pose_end3.fill(0.0);
    std::array<double, 16> pose_end4;
    pose_end4.fill(0.0);
    std::array<double, 16> pose_end5;
    pose_end5.fill(0.0);

    std::vector<std::array<double, 16>> pos_end_vector;
    std::array<double, 16> pose_start;
    CartesianPose output{};

    CartesianControl cartesian_position_callback;
    cartesian_position_callback = [&](RCI::robot::RobotState robot_state) -> CartesianPose {
        // std::cout<<"rate:"<<robot_state.control_command_success_rate<<std::endl;
        time += 0.001; 
        if(init==true){
            init_position = robot_state.toolTobase_pos_m;
            pose_start = init_position;
            pose_end1 = init_position;
            pose_end1[11] -= 0.2;
            pose_end2 = pose_end1;
            pose_end2[7] -= 0.1;
            pose_end3 = pose_end2;
            pose_end3[11] += 0.2;
            pose_end4 = pose_end3;
            pose_end4[7] += 0.1;
            pos_end_vector.push_back(pose_end1);
            pos_end_vector.push_back(pose_end2);
            pos_end_vector.push_back(pose_end3);
            pos_end_vector.push_back(pose_end4);
            init=false;
        }
        std::array<double, 16> pose_start = init_position;
        pose_end = pos_end_vector[count];

        Eigen::Vector3d pos_1(pose_start[3], pose_start[7], pose_start[11]);
        Eigen::Vector3d pos_2(pose_end[3], pose_end[7], pose_end[11]);

        Eigen::Vector3d pos_delta = pos_2 - pos_1;
        double s = pos_delta.norm();
        Eigen::Vector3d pos_delta_vector = pos_delta.normalized();

        CartMotionGenerator cart_s(0.05, s);
        cart_s.calculateSynchronizedValues_cart(0);

        Eigen::Matrix3d mat_start, mat_end;
        mat_start << pose_start[0], pose_start[1], pose_start[2], pose_start[4], pose_start[5], pose_start[6],
            pose_start[8], pose_start[9], pose_start[10];

        mat_end << pose_end[0], pose_end[1], pose_end[2], pose_end[4], pose_end[5], pose_end[6], pose_end[8],
            pose_end[9], pose_end[10];

        Eigen::Quaterniond rot_start(mat_start);
        Eigen::Quaterniond rot_end(mat_end);

        Eigen::Vector3d pos_cur;

        if (cart_s.calculateDesiredValues_cart(time, &delta_s) == false) {
            pos_cur = pos_1 + pos_delta * delta_s / s;
            Eigen::Quaterniond rot_cur = rot_start.slerp(delta_s / s, rot_end);
            mat_cur = rot_cur.normalized().toRotationMatrix();
            std::array<double, 16> new_pose = {
                {mat_cur(0, 0), mat_cur(0, 1), mat_cur(0, 2), pos_cur(0), mat_cur(1, 0), mat_cur(1, 1),
                    mat_cur(1, 2), pos_cur(1), mat_cur(2, 0), mat_cur(2, 1), mat_cur(2, 2), pos_cur(2), 0, 0, 0, 1}};
            output.toolTobase_pos_c = new_pose;
        }else {
            count++;
            std::cout << "第" <<count<<"段直线" << std::endl;
            init_position = pos_end_vector[count-1];
            time = 0;
            if (count >= pos_end_vector.size()) {
                std::cout<<"运动结束"<<std::endl;
                return MotionFinished(output);
            }

        }
        return output;        
    };
    robot.Control(cartesian_position_callback);

    return 0;
}
