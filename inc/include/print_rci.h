/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#pragma once

#include <iostream>
#include <iterator>

#include "rci_data/robot_datas.h"

namespace xmate {
//打印std::array
template <class T, size_t N>
inline std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array) {
    ostream << "[";
    std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
    std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
}
//打印RobotMode
inline std::ostream &operator<<(std::ostream &ostream, const RCI::robot::RobotMode robot_mode) {
    ostream << "\"";
    switch (robot_mode) {
        case (RCI::robot::RobotMode::kUserStopped):
            ostream << "User stopped";
            break;
        case (RCI::robot::RobotMode::kIdle):
            ostream << "Idle";
            break;
        case (RCI::robot::RobotMode::kMove):
            ostream << "Move";
            break;
        case (RCI::robot::RobotMode::kGuiding):
            ostream << "Guiding";
            break;
        case (RCI::robot::RobotMode::kReflex):
            ostream << "Reflex";
            break;
        case (RCI::robot::RobotMode::kAutomaticErrorRecovery):
            ostream << "Automatic error recovery";
            break;
        case (RCI::robot::RobotMode::kOther):
            ostream << "Other";
            break;
    }
    ostream << "\"";
    return ostream;
}
//打印ControllerMode
inline std::ostream &operator<<(std::ostream &ostream, const RCI::robot::ControllerMode control_mode) {
    ostream << "\"";
    switch (control_mode) {
        case (RCI::robot::ControllerMode::kJointPosition):
            ostream << "kJointPosition";
            break;
        case (RCI::robot::ControllerMode::kCartesianPosition):
            ostream << "kCartesianPosition";
            break;
        case (RCI::robot::ControllerMode::kJointImpedance):
            ostream << "kJointImpedance";
            break;
        case (RCI::robot::ControllerMode::kCartesianImpedance):
            ostream << "kCartesianImpedance";
            break;
        case (RCI::robot::ControllerMode::kTorque):
            ostream << "kTorque";
            break;
        case (RCI::robot::ControllerMode::kOther):
            ostream << "kOther";
            break;
    }
    ostream << "\"";
    return ostream;
}
// MotionGeneratorMode
inline std::ostream &operator<<(std::ostream &ostream, const RCI::robot::MotionGeneratorMode motion_mode) {
    ostream << "\"";
    switch (motion_mode) {
        case (RCI::robot::MotionGeneratorMode::kIdle):
            ostream << "kIdle";
            break;
        case (RCI::robot::MotionGeneratorMode::kJointPosition):
            ostream << "kJointPosition";
            break;
        case (RCI::robot::MotionGeneratorMode::kJointVelocity):
            ostream << "kJointVelocity";
            break;
        case (RCI::robot::MotionGeneratorMode::kCartesianPosition):
            ostream << "kCartesianPosition";
            break;
        case (RCI::robot::MotionGeneratorMode::kCartesianVelocity):
            ostream << "kCartesianVelocity";
            break;
    }
    ostream << "\"";
    return ostream;
}

//打印RobotState
inline std::ostream &operator<<(std::ostream &ostream, const RCI::robot::RobotState &robot_state) {
    ostream << "{\"q\": " << robot_state.q << ", \"q_c\": " << robot_state.q_c << ", \"dq_m\": " << robot_state.dq_m << ", \"dq_c\": " << robot_state.dq_c
            << ", \"ddq_c\": " << robot_state.ddq_c << ", \"toolTobase_pos_m\": " << robot_state.toolTobase_pos_m
            << ", \"toolTobase_pos_c\": " << robot_state.toolTobase_pos_c << ", \"toolTobase_vel_c\": " << robot_state.toolTobase_vel_c
            << ", \"toolTobase_acc_c\": " << robot_state.toolTobase_acc_c << ", \"psi_m\": " << robot_state.psi_m << ", \"psi_c\": " << robot_state.psi_c
            << ", \"psi_vel_c\": " << robot_state.psi_vel_c << ", \"psi_acc_c\": " << robot_state.psi_acc_c << ", \"tau_m\": " << robot_state.tau_m
            << ", \"tau_m_filtered\": " << robot_state.tau_m_filtered << ", \"tau_c\": " << robot_state.tau_c << ", \"tau_vel_c\": " << robot_state.tau_vel_c
            << ", \"tau_ext_in_base\": " << robot_state.tau_ext_in_base << ", \"tau_ext_in_stiff\": " << robot_state.tau_ext_in_stiff
            << ", \"theta_m\": " << robot_state.theta_m << ", \"theta_vel_m\": " << robot_state.theta_vel_m << ", \"motor_tau\": " << robot_state.motor_tau
            << ", \"motor_tau_filtered\": " << robot_state.motor_tau_filtered << ", \"errors\": " << robot_state.errors
            << ", \"control_command_success_rate\": " << robot_state.control_command_success_rate
            << ", \"motion_generator_mode\": " << robot_state.motion_generator_mode << ", \"controller_mode\": " << robot_state.controller_mode
            << ", \"robot_mode\": " << robot_state.robot_mode << "}";
    return ostream;
}

//打印RobotCommand
inline std::ostream &operator<<(std::ostream &ostream, const RCI::robot::RobotCommand &robot_command) {
    ostream << "{\"q_c\": " << robot_command.motion.q_c << ", \"toolTobase_pos_c\": " << robot_command.motion.toolTobase_pos_c
            << ", \"psi_vaild\": " << robot_command.motion.psi_valid << ", \"psi_c\": " << robot_command.motion.psi_c
            << ", \"motion_generation_finished\": " << robot_command.motion.motion_generation_finished << ", \"tau_c\": " << robot_command.torque.tau_c << "}";
    return ostream;
}
}  // namespace xmate