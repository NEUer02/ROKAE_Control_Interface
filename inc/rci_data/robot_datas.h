/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include "./platform.h"

#ifdef _WIN32
#include <WinSock2.h>
#endif  // _WIN32

#ifdef _linux_
#include <arpa/inet.h>
#endif  //_linux_

// #include <arpa/inet.h>

#include <array>
#ifdef HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif

constexpr long unsigned int RCI_JOINT_NUM = 7;

//#include <cstdint>

namespace RCI {
namespace robot {
inline uint64_t htonll(uint64_t val) { return (((uint64_t)htonl((uint32_t)val)) << 32) + htonl((uint32_t)(val >> 32)); }

inline uint64_t ntohll(uint64_t val) { return (((uint64_t)ntohl((uint32_t)val)) << 32) + ntohl((uint32_t)(val >> 32)); }

#pragma pack(push, 1)

enum class MotionGeneratorMode : uint8_t { kIdle, kJointPosition, kJointVelocity, kCartesianPosition, kCartesianVelocity };

enum class ControllerMode : uint8_t { kJointPosition, kCartesianPosition, kJointImpedance, kCartesianImpedance, kTorque, kOther };

enum class RobotMode : uint8_t { kOther, kIdle, kMove, kGuiding, kReflex, kUserStopped, kAutomaticErrorRecovery };

/**
 * 机器人控制器发送给RCI的机器人状态
 */

struct RobotState {
    uint64_t message_id;
    std::array<double, 7> q{};                  //关节角度
    std::array<double, 7> q_c{};                //指令关节角度
    std::array<double, 7> dq_m{};               //关节速度
    std::array<double, 7> dq_c{};               //指令关节速度
    std::array<double, 7> ddq_c{};              //指令关节加速度
    std::array<double, 16> toolTobase_pos_m{};  //机器人位姿
    std::array<double, 16> toolTobase_pos_c{};  //指令机器人位姿
    std::array<double, 6> toolTobase_vel_c{};   //指令机器人末端速度
    std::array<double, 6> toolTobase_acc_c{};   //指令机器人末端加速度
    double psi_m;                               //臂角
    double psi_c;                               //指令臂角
    double psi_vel_c;                           //指令臂角速度
    double psi_acc_c;                           //指令臂角加速度
    std::array<double, 7> tau_m{};              //关节力矩
    std::array<double, 7> tau_m_filtered;
    std::array<double, 7> tau_c{};             //指令关节力矩
    std::array<double, 7> tau_vel_c{};         //指令力矩微分
    std::array<double, 6> tau_ext_in_base{};   //基坐标系中外部力矩
    std::array<double, 6> tau_ext_in_stiff{};  //力控坐标系中外部力矩
    std::array<double, 7> theta_m{};           //电机位置
    std::array<double, 7> theta_vel_m{};       //电机位置微分
    std::array<double, 7> motor_tau{};         //电机转矩
    std::array<double, 7> motor_tau_filtered;
    std::array<bool, 20> errors{};              //错误位
    double control_command_success_rate{};      //指令接收成功率
    MotionGeneratorMode motion_generator_mode;  //运动发生模式
    ControllerMode controller_mode;             //控制模式
    RobotMode robot_mode;                       //机器人状态

    void HostToNet() { message_id = htonll(message_id); }

    void NetToHost() { message_id = ntohll(message_id); }
};

struct MotionCommand {
    std::array<double, 7> q_c;                //关节角度
    std::array<double, 16> toolTobase_pos_c;  //末端位姿
    bool psi_valid;                           //是否有臂角
    double psi_c;                             //臂角
    bool motion_generation_finished;          //运动是否结束
    std::array<double, 4> dynamic_wall;

    void HostToNet() {}
    void NetToHost() {}
};

struct TorqueCommand {
    std::array<double, 7> tau_c;  //关节力矩

    void HostToNet() {}
    void NetToHost() {}
};

/**
 * RCI给机器人控制器发送的指令
 */
struct RobotCommand {
    uint64_t message_id;
    MotionCommand motion;
    TorqueCommand torque;

    void HostToNet() { message_id = htonll(message_id); }

    void NetToHost() { message_id = ntohll(message_id); }
};

#pragma pack(pop)

}  // namespace robot
}  // namespace RCI
