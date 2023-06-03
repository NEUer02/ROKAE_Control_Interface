/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <Eigen/Core>
#include <array>
#include <iostream>

#include "control_types.h"
#include "duration.h"
#include "rci_data/robot_datas.h"
#include "robot.h"

/**
 * S速度规划的轴空间运动. 参考文献:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 *
 */

using VectorNd = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
using VectorNi = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

class JointMotionGenerator {
   public:
    /**
     * 根据关节目标位置和速度系数生成一条轴空间轨迹，可用来回零或到达指定位置.
     *
     * @param[in] 速度系数，范围[0, 1].
     * @param[in] 目标关节角度.
     */
    JointMotionGenerator(double speed_factor, const std::array<double, 7> q_goal);
    ~JointMotionGenerator() {}
    /**
     * 设置轴空间运动最大速度，最大开始加速度，最大结束加速度
     */

    bool setMax(std::array<double, 7> dq_max, std::array<double, 7> ddq_max_start, std::array<double, 7> ddq_max_end);

    /**
     * 获得总运动时间
     */
    inline double GetTime() { return t_f_sync_.maxCoeff(); }

    /**
     * 计算时间t时的关节角度增量
     * @return false代表运动规划没有结束，true代表运动规划结束
     */

    bool calculateDesiredValues_joint(double t, std::array<double, 7> &delta_q_d) const;

    void calculateSynchronizedValues_joint(const std::array<double, 7> &q_init);

   private:
    static constexpr double kDeltaQMotionFinished = 1e-6;
    const VectorNd q_goal_;

    double speed_factor_;

    VectorNd q_start_;
    VectorNd delta_q_;

    VectorNd dq_max_sync_;
    VectorNd t_1_sync_;
    VectorNd t_2_sync_;
    VectorNd t_f_sync_;
    VectorNd q_1_;

    double time_ = 0.0;

    VectorNd dq_max_ = (VectorNd() << 1, 1, 1, 1, 1.25, 1.25, 1.25).finished();
    VectorNd ddq_max_start_ = (VectorNd() << 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5).finished();
    VectorNd ddq_max_goal_ = (VectorNd() << 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5).finished();
};
