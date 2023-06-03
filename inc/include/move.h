/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#include <array>
#include <iostream>

#include "cart_motion.h"
#include "joint_motion.h"
#include "robot.h"

enum class MOVETYPE { MOVEJ, MOVEL, MOVEC };

struct cart_pos {
    std::array<double, 16> pos;  //行优先，位姿矩阵
    bool psi_valid;
    double psi;  //臂角

    cart_pos() {
        pos.fill(0.0);
        psi_valid = false;
        psi       = 0.0;
    }
};

/**
 * @brief MOVEJ指令,在到达q_target之前处于阻塞状态。q_start需要是机器人当前关节角度，否则可能造成下电。
 *
 * @param[in] 速度比例系数
 * @param[in] 机器人开始关节角度，
 * @param[in] 机器人目标关节角度
 * @param[in] robot对象
 */
bool MOVEJ(double speed, const std::array<double, 7>& q_start, const std::array<double, 7>& q_target, xmate::Robot& robot);

/**
 * @brief MOVEL指令，在到达pos_target之前处于处于阻塞状态。pos_start需要是机器人当前位姿（如果设置了TCP，那么应该是工具相对于基坐标系的位姿），
 * 否则可能造成下电，同理，pos_target也应该是TCP相对于基坐标系的位姿。
 *
 * @param[in] 速度比例系数
 * @param[in] 机器人开始位姿
 * @param[in] 机器人目标位姿
 * @param[in] robot对象
 */
bool MOVEL(double speed, cart_pos& pos_start, cart_pos& pos_targrt, xmate::Robot& robot);

/**
 * @brief MOVEC指令，在到达pos_target之前处于阻塞状态。pos_start需要是机器人当前位姿（如果设置了TCP，那么应该是工具相对于基坐标系的位姿），
 * 否则可能造成下电，同理，pos_mid, pos_target也应该是TCP相对于基坐标系的位姿。
 *
 * @param[in] 速度比例系数
 * @param[in] 机器人开始位姿
 * @param[in] 机器人中间位姿
 * @param[in] 机器人目标位姿
 * @param[in] robot对象
 */
bool MOVEC(double speed, cart_pos& pos_start, cart_pos& pos_mid, cart_pos& pos_target, xmate::Robot& robot);
