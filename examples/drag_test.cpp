/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 开启拖动示例
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
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1338;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    
    // RCI连接机器人
    xmate::Robot robot(ipaddr, port,XmateType::XMATE3);
    //防止网络连接失败
    sleep(2);
    int res = robot.getMotorState();
    std::cout<<"机器人上电状态："<<res<<std::endl;
    int power_state=1;
    robot.setMotorPower(power_state);
    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,PI/3,0,PI/2,0,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);
    sleep(2);

    // robot.setLoad(1.0,{{0.0,0.0,0}},{{0,0,0,0,0,0,0,0,0}});
    double angle = 0;
    std::array<double, 16> coor_array = {{cos(angle), 0, sin(angle), 0, 0, 1, 0, 0.0, -sin(angle), 0, cos(angle), 0.2, 0, 0, 0, 1}};
    robot.setCoor(coor_array);

    robot.startDrag(RCI::robot::DragSpace::CARTESIAN_DRAG, RCI::robot::DragType::FREELY);

    //主线程sleep 100s后发送停止拖动接口
    sleep(100);
    robot.stopDrag();

    return 0;
}
