/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <array>
#include <ostream>
#include <string>

/**
 * @file errors.h
 * Contains the xmate::Errors type.
 */

namespace xmate {

/**
 * 列举运动过程中的错误
 */
struct Errors {
   public:
    std::array<bool, 20> errors_{};

   public:
    /**
     * 创建一个空Error对象
     */
    Errors();

    Errors(const Errors &other);

    Errors &operator=(Errors other);

    Errors(const std::array<bool, 20> &errors);

    /**
     * 判断是否有任何一个错误位置为true
     */
    explicit operator bool() const noexcept;

    /**
     * 将错误转换成字符串
     *
     * @return 错误字符
     */
    explicit operator std::string() const;

    const bool &kActualJointPositionLimitsViolation;
    const bool &kActualCartesianPositionLimitsViolation;
    const bool &kActualCartesianMotionGeneratorElbowLimitViolation;
    const bool &kActualJointVelocityLimitsViolation;
    const bool &kActualCartesianVelocityLimitsViolation;
    const bool &kActualJointAccelerationLimitsViolation;
    const bool &kActualCartesianAccelerationLimitsViolation;
    const bool &kCommandJointPositionLimitsViolation;
    const bool &kCommandCartesianPositionLimitsViolation;
    const bool &kCommandCartesianMotionGeneratorElbowLimitViolation;
    const bool &kCommandJointVelocityLimitsViolation;
    const bool &kCommandCartesianVelocityLimitsViolation;
    const bool &kCommandJointAccelerationLimitsViolation;
    const bool &kCommandCartesianAccelerationLimitsViolation;
    const bool &kCommandJointAccelerationDiscontinuity;
    const bool &kCollision;
    const bool &kCartesianPositionMotionGeneratorInvalidFrame;
    const bool &kCommandTorqueDiscontinuity;
    const bool &kCommandTorqueRangeViolation;
    const bool &kInstabilityDetection;
};

/**
 * Streams the errors as JSON array.
 *
 * @return Ostream instance
 */
std::ostream &operator<<(std::ostream &ostream, const Errors &errors);

}  // namespace xmate
