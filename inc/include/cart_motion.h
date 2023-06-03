/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <Eigen/Core>
#include <array>

#include "control_types.h"
#include "duration.h"
#include "robot.h"

/**
 * S速度规划的笛卡尔空间运动. 参考文献:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class CartMotionGenerator {
   public:
    /**
     * 根据关节目标位置和速度系数生成一条轴空间轨迹，可用来回零或到达指定位置.
     *
     * @param[in] 速度系数，范围[0, 1].
     * @param[in] 目标关节角度.
     */

    CartMotionGenerator(double speed_factor, const double s_goal);

    /**
     * 设置笛卡尔空间运动最大速度，最大开始加速度，最大结束加速度
     */

    bool setMax(double ds_max, double dds_max_start, double dds_max_end);

    /**
     * 获得总运动时间
     */

    inline double GetTime() { return t_f_sync_; }

    /**
     * 计算时间t时的弧长s
     * @return false代表运动规划没有结束，true代表运动规划结束
     */

    bool calculateDesiredValues_cart(double t, double *delta_s_d) const;

    /**
     * s规划，笛卡尔空间是弧长s规划
     */
    void calculateSynchronizedValues_cart(double s_init);

   private:
    static constexpr double kDeltaQMotionFinished = 1e-6;
    const double s_goal_;

    double speed_factor_;

    double s_start_;
    double delta_s_;

    double ds_max_sync_;
    double t_1_sync_;
    double t_2_sync_;
    double t_f_sync_;
    double s_1_;

    double time_ = 0.0;

    double ds_max_        = 1.0;
    double dds_max_start_ = 2.5;
    double dds_max_goal_  = 2.5;
};
