/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <array>
#include <limits>
#include <cmath>
#include <vector>

namespace xmate {

constexpr double deg2rad = M_PI/180.0;
/**
 * 采样周期
 */
constexpr double kDeltaT = 1e-3;

constexpr double kLimitEps = 1e-3;

constexpr double kNormEps = std::numeric_limits<double>::epsilon();
/**
 * 在计算速度极限时用到的丢包数量
 */
constexpr double kTolNumberPacketsLost = 3.0;

constexpr double kFactorCartesianRotationPoseInterface = 0.99;
/**
 * 最大额定力矩
 */

constexpr std::array<double, 7> kMaxTorqueRate{
    {85 - kLimitEps, 85 - kLimitEps, 85 - kLimitEps, 85 - kLimitEps, 36 - kLimitEps, 36 - kLimitEps, 36 - kLimitEps}};
/**
 * 最大关节加加速度
 */
constexpr std::array<double, 7> kMaxJointJerk{{5000 - kLimitEps, 3500 - kLimitEps, 5000  - kLimitEps,
                                               5000  - kLimitEps, 7500  - kLimitEps, 7500 - kLimitEps,
                                               7500  - kLimitEps}};
/**
 * 最大关节加速度
 */
constexpr std::array<double, 7> kMaxJointAcceleration{{15  - kLimitEps, 7.5 - kLimitEps, 10  - kLimitEps,
                                                       10  - kLimitEps, 15  - kLimitEps, 15  - kLimitEps,
                                                       20  - kLimitEps}};
/**
 * 最大关节速度
 */
constexpr std::array<double, 7> kMaxJointVelocity{
    {(2.175  - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxJointAcceleration[0]),
     (2.175  - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[1]),
     (2.175  - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[2]),
     (2.175  - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[3]),
     (2.610  - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[4]),
     (2.610  - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[5]),
     (2.610  - kLimitEps - kTolNumberPacketsLost* kDeltaT* kMaxJointAcceleration[6])}};
/**
 * 最大平移加加速度
 */
constexpr double kMaxTranslationalJerk = 5000.0 - kLimitEps;
/**
 * 最大平移加速度
 */
constexpr double kMaxTranslationalAcceleration = 10.0 - kLimitEps;
/**
 * 最大平移速度
 */
constexpr double kMaxTranslationalVelocity =
    1.5 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxTranslationalAcceleration;
/**
 * 最大旋转加加速度
 */
constexpr double kMaxRotationalJerk = 12500.0 - kLimitEps;
/**
 * 最大旋转加速度
 */
constexpr double kMaxRotationalAcceleration = 25.0 - kLimitEps;
/**
 * 最大旋转速度
 */
constexpr double kMaxRotationalVelocity =
    2.5 - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxRotationalAcceleration;
/**
 * 最大臂角加加速度
 */
constexpr double kMaxElbowJerk = 5000 - kLimitEps;
/**
 * 最大臂角加速度
 */
constexpr double kMaxElbowAcceleration = 10 - kLimitEps;
/**
 * Maximum 最大臂角速度
 */
constexpr double kMaxElbowVelocity = 180*deg2rad - kLimitEps - kTolNumberPacketsLost * kDeltaT * kMaxElbowAcceleration;

/**
 * 用于对关节力矩值进行限幅.
 *
 *
 * @param[in] 允许的最大指令微分值.
 * @param[in] 指令值.
 * @param[in] 前一个周期的指令值.
 * @return 限幅后的指令值
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_derivatives,
                                const std::array<double, 7>& commanded_values,
                                const std::array<double, 7>& last_commanded_values);

/**
 * 用于对笛卡尔速度进行限幅.
 *
 * @param[in] 允许的最大笛卡尔速度.
 * @param[in] 允许的最大笛卡尔加速度.
 * @param[in] 允许的最大笛卡尔加加速度.
 * @param[in] 速度指令值.
 * @param[in] 前一个周期的笛卡尔速度指令值.
 * @param[in] 前一个周期的笛卡尔加速度指令值.
 * @return 限幅的指令值
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 */
double limitRate(double max_velocity, double max_acceleration, double max_jerk, double commanded_velocity,
                 double last_commanded_velocity, double last_commanded_acceleration);

/**
 * 用于对笛卡尔位置指令值进行限幅.
 *
 * @param[in] 允许的最大速度.
 * @param[in] 允许的最大加速度.
 * @param[in] 允许的最尔加加速度.
 * @param[in] 位置指令值.
 * @param[in] 前一个周期的位置指令值.
 * @param[in] 前一个周期的速度指令值.
 * @param[in] 前一个周期的加速度指令值.
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 */
double limitRate(double max_velocity, double max_acceleration, double max_jerk, double commanded_position,
                 double last_commanded_position, double last_commanded_velocity, double last_commanded_acceleration);

/**
 * 用于对关节速度进行限幅.
 *
 * @param[in] 允许的最大关节速度.
 * @param[in] 允许的最大关节加速度.
 * @param[in] 允许的最大关节加加速度.
 * @param[in] 速度指令值.
 * @param[in] 前一个周期的速度指令值.
 * @param[in] 前一个周期的加速度指令值.
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration, const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& commanded_velocities,
                                const std::array<double, 7>& last_commanded_velocities,
                                const std::array<double, 7>& last_commanded_accelerations);

/**
 * 用于对关节位置指令值进行限幅.
 *
 * @param[in] 允许的最大关节速度.
 * @param[in] 允许的最大关节加速度.
 * @param[in] 允许的最大关节加加速度.
 * @param[in] 关节位置指令值.
 * @param[in] 前一个周期的关节位置指令值.
 * @param[in] 前一个周期的关节速度指令值.
 * @param[in] 前一个周期的关节加速度指令值.
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 */
std::array<double, 7> limitRate(const std::array<double, 7>& max_velocity,
                                const std::array<double, 7>& max_acceleration, const std::array<double, 7>& max_jerk,
                                const std::array<double, 7>& commanded_positions,
                                const std::array<double, 7>& last_commanded_positions,
                                const std::array<double, 7>& last_commanded_velocities,
                                const std::array<double, 7>& last_commanded_accelerations);

/**
 * 用于对笛卡尔速度指令值进行限幅.
 *
 * @param[in] 允许的最大笛卡尔平移速度.
 * @param[in] 允许的最大笛卡尔平移加速度.
 * @param[in] 允许的最大笛卡尔平移加加速度.
 * @param[in] 允许的最大笛卡尔旋转速度.
 * @param[in] 允许的最大笛卡尔旋转加速度.
 * @param[in] 允许的最大笛卡尔旋转加加速度.
 * @param[in] 前一个周期的笛卡尔位置指令值.
 * @param[in] 前一个周期的笛卡尔速度指令值.
 * @param[in] 前一个周期的笛卡尔加速度指令值.
 *
 * @return 限幅后的笛卡尔速度指令值.
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 */
std::array<double, 6> limitRate(double max_translational_velocity, double max_translational_acceleration,
                                double max_translational_jerk, double max_rotational_velocity,
                                double max_rotational_acceleration, double max_rotational_jerk,
                                const std::array<double, 6>& toolTobase_vel_c,
                                const std::array<double, 6>& last_O_dP_EE_c,
                                const std::array<double, 6>& last_O_ddP_EE_c);

/**
 * 用于对笛卡尔速度指令值进行限幅.
 *
 * @param[in] 允许的最大笛卡尔平移速度.
 * @param[in] 允许的最大笛卡尔平移加速度.
 * @param[in] 允许的最大笛卡尔平移加加速度.
 * @param[in] 允许的最大笛卡尔旋转速度.
 * @param[in] 允许的最大笛卡尔旋转加速度.
 * @param[in] 允许的最大笛卡尔旋转加加速度.
 * @param[in] 此刻笛卡尔位置指令值.
 * @param[in] 前一个周期的笛卡尔位置指令值.
 * @param[in] 前一个周期的笛卡尔速度指令值.
 * @param[in] 前一个周期的笛卡尔加速度指令值.
 *
 * @return 限幅后的笛卡尔位置指令值.
 *
 * @throw std::invalid_argument if commanded_values are infinite or NaN.
 *
 */
std::array<double, 16> limitRate(double max_translational_velocity, double max_translational_acceleration,
                                 double max_translational_jerk, double max_rotational_velocity,
                                 double max_rotational_acceleration, double max_rotational_jerk,
                                 const std::array<double, 16>& toolTobase_pos_c,
                                 const std::array<double, 16>& last_O_T_EE_c,
                                 const std::array<double, 6>& last_O_dP_EE_c,
                                 const std::array<double, 6>& last_O_ddP_EE_c);

}  // namespace xmate
