/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <array>
#include <cmath>

/**
 * @file lowpass_filter.h
 * 低通滤波器
 */

namespace xmate {
/**
 * 最大截止频率
 */
constexpr double kMaxCutoffFrequency = 1000.0;
/**
 * 默认截止频率
 */
constexpr double kDefaultCutoffFrequency = 100.0;
/**
 * 一阶低通滤波器
 *
 * @param[in] 采样时间
 * @param[in] 需要滤波的数据
 * @param[in] 上一周期的滤波数据
 * @param[in] 截止频率
 *
 * @throw std::invalid_argument if y is infinite or NaN.
 * @throw std::invalid_argument if y_last is infinite or NaN.
 * @throw std::invalid_argument if cutoff_frequency is zero, negative, infinite or NaN.
 * @throw std::invalid_argument if sample_time is negative, infinite or NaN.
 *
 * @return 滤波后的数据.
 */
double lowpassFilter(double sample_time, double y, double y_last, double cutoff_frequency);

/**
 * 笛卡尔空间位姿的一阶低通滤波器
 *
 * @param[in] 采样时间
 * @param[in] 需要滤波的数据
 * @param[in] 上一周期的滤波数据
 * @param[in] 截止频率
 *
 * @throw std::invalid_argument if y is infinite or NaN.
 * @throw std::invalid_argument if y_last is infinite or NaN.
 * @throw std::invalid_argument if cutoff_frequency is zero, negative, infinite or NaN.
 * @throw std::invalid_argument if sample_time is negative, infinite or NaN.
 *
 * @return 滤波后的数据.
 */

std::array<double, 16> cartesianLowpassFilter(double sample_time, std::array<double, 16> y, std::array<double, 16> y_last, double cutoff_frequency);
}  // namespace xmate
