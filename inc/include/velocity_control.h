/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <iostream>
#include <array>
#include <cmath>

#ifdef _WIN32
    const double M_PI = 3.141592653;
#endif

/**
 * 在线速度规划器
 */

class VelocityApply
{
public:
    VelocityApply(const int joint_num);
    ~VelocityApply(){};

public:
    int Init(const std::array<double, 7> &q,//机器人开始做运动规划时的角度
             const double delta_t,
             const std::array<double, 7> q_min,
             const std::array<double, 7> q_max,
             const std::array<double, 7> dq_max,
             const std::array<double, 7> ddq_max);
    /**
     * 在线规划和平滑轨迹，注意：gain值太高可能会造成不稳定，gain建议取值0~15
     * @param[in] 期望速度
     * @param[out] 位置指令
     * @param[out] 速度指令
     */
    void Apply(const double gain, std::array<double, 7> &dq_des,
               std::array<double, 7> &q_cmd,
               std::array<double, 7> &dq_cmd);

private:
    typedef enum
    {
        FLAGACC, // Axis in acceleration
        FLAGCTE, // Axis at constant velocity
        FLAGDEC, // Axis in deceleration
        FLAGSTO  // Axis stopped
    } status_t;
    size_t njoints_;
    double delta_t_;

    std::array<double, 7> q_min_;   // Joint min position
    std::array<double, 7> q_max_;   // Joint max position
    std::array<double, 7> dq_max_;  // Joint max velocity
    std::array<double, 7> ddq_max_; // Joint max acceleration

    std::array<double, 7> q_final_;                        // Final position before joint limit 安全位置限制距离
    std::array<double, 7> q_cmd_;                          // Joint position command
    std::array<double, 7> q_cmd_prev_;                     // Previous joint position command
    std::array<double, 7> dq_des_;                         // Desired velocity  期望关节速度
    std::array<double, 7> dq_des_prev_;                    // Previous desired velocity  上一次的期望关节速度
    std::array<double, 7> delta_q_;                        // Current position increment 当前周期的位置增量
    std::array<double, 7> delta_q_max_;                    // Max position increment  当前周期的最大位置增量
    const double delta_q_min_ = 1e-9;                      // Delta q minimum (rad)
    std::array<int, 7> sign_;                              // Displacement sign: +1 = increment position 速度方向，正向或反向
    std::array<double, 7> dist_to_final_;                  // Distance between current joint position and final
    std::array<double, 7> dist_ad_;                        // Distance required to accelerate or decelerate
    std::array<bool, 7> flag_speed_;                       // Change of speed direction  速度的方向是否改变
    std::array<status_t, 7> status_;                       // Axis status
    std::array<double, 7> delta_q_acc_;                    // Increment related to acceleration 以最大加速度运行单个周期的距离增量
    bool flag_joint_limit_;                                // 是否关节被限制运动
    const double offset_joint_limit_ = 1.0 / 180.0 * M_PI; // stop before joint limit (rad) 在关节限制之前一段距离停止
};