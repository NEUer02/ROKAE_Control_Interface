/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include "rci_data/robot_datas.h"

#include <array>
#include <cmath>
#include <initializer_list>

/**
 * @file control_types.h
 * 包含运动过程中一个控制周期内RCI与控制器交互的运动数据
 */

namespace xmate {

/**
 * 控制器控制模式
 */
enum class ControllerMode { kJointPosition, kCartesianPosition, kJointImpedance, kCartesianImpedance, kTorque };

/**
 * 用于判断是否结束一个运动循环，即是否终止回调
 */
struct Finishable {
    bool motion_finished = false;
};

/**
 * 不包含重力和摩擦力的关节力矩
 */
class Torques : public Finishable {
   public:
    /**
     * 创建一个关节力矩对象
     *
     * @param[in] 不包含重力和摩擦力的关节力矩 [Nm].
     */
    Torques(const std::array<double, 7>& torques) noexcept;

    /**
     * 创建一个新的关节力矩对象
     *
     * @param[in] 不包含重力和摩擦力的关节力矩 [Nm].
     *
     * @throw 参数个数异常
     */
    Torques(std::initializer_list<double> torques);

    Torques();

    /**
     * 期望关节力矩 [Nm].
     */
    std::array<double, 7> tau_c{};
};

/**
 * 关节位置
 */
class JointPositions : public Finishable {
   public:
    /**
     * 创建一个关节位置对象
     *
     * @param[in] 关节角度 [rad].
     */
    JointPositions(const std::array<double, 7>& joint_positions) noexcept;

    /**
     * 创建一个新的关节位置对象
     *
     * @param[in] 关节角度 [rad].
     *
     * @throw 参数个数异常.
     */
    JointPositions(std::initializer_list<double> joint_positions);

    JointPositions();

    /**
     * 期望关节角度 [rad].
     */
    std::array<double, 7> q{};
};

/**
 * 笛卡尔空间位置
 */
class CartesianPose : public Finishable {
   public:
    /**
     * 创建一个笛卡尔空间位姿态的对象
     *
     * @param[in] 末端执行器相对与基坐标系的齐次变换矩阵 column major.
     */
    CartesianPose(const std::array<double, 16>& cartesian_pose) noexcept;

    /**
     * 创建一个笛卡尔空间位姿态的对象
     *
     * @param[in] 末端执行器相对与基坐标系的齐次变换矩阵 column major
     * @param[in] 臂角
     */
    CartesianPose(const std::array<double, 16>& cartesian_pose, const double& psi_m) noexcept;

    /**
     * 创建一个笛卡尔空间位姿态的对象
     *
     * @param[in] 末端执行器相对与基坐标系的齐次变换矩阵 column major.
     *
     * @throw 参数个数异常
     */
    CartesianPose(std::initializer_list<double> cartesian_pose);

    CartesianPose();

    /**
     * 创建一个笛卡尔空间位姿态的对象
     *
     * @param[in] 末端执行器相对与基坐标系的齐次变换矩阵 column major
     * @param[in] 臂角
     *
     * @throw 参数个数异常
     */
    CartesianPose(std::initializer_list<double> cartesian_pose, double psi_c);

    /**
     *末端执行器相对与基坐标系的齐次变换矩阵
     */
    std::array<double, 16> toolTobase_pos_c{};

    /**
     * 臂角
     */
    double psi_c{};

    /**
     * 是否配置臂角
     */

    bool hasElbow() const noexcept;
};

inline Torques MotionFinished(Torques command) noexcept {
    command.motion_finished = true;
    return command;
}

inline JointPositions MotionFinished(JointPositions command) noexcept {
    command.motion_finished = true;
    return command;
}

inline CartesianPose MotionFinished(CartesianPose command) noexcept {
    command.motion_finished = true;
    return command;
}

}  // namespace xmate
