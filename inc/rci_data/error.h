/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 */
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace RCI {
namespace robot {

enum class Error : size_t {
    kActualJointPositionLimitsViolation,
    kActualCartesianPositionLimitsViolation,
    kActualCartesianMotionGeneratorElbowLimitViolation,
    kActualJointVelocityLimitsViolation,
    kActualCartesianVelocityLimitsViolation,
    kActualJointAccelerationLimitsViolation,
    kActualCartesianAccelerationLimitsViolation,
    kCommandJointPositionLimitsViolation,
    kCommandCartesianPositionLimitsViolation,
    kCommandCartesianMotionGeneratorElbowLimitViolation,
    kCommandJointVelocityLimitsViolation,
    kCommandCartesianVelocityLimitsViolation,
    kCommandJointAccelerationLimitsViolation,
    kCommandCartesianAccelerationLimitsViolation,
    kCommandJointAccelerationDiscontinuity,
    kCollision,
    kCartesianPositionMotionGeneratorInvalidFrame,
    kCommandTorqueDiscontinuity,
    kCommandTorqueRangeViolation,
    kInstabilityDetection
};

const char *getErrorName(Error error) {
    switch (error) {
        case Error::kActualJointPositionLimitsViolation:
            return "kActualJointPositionLimitsViolation";
        case Error::kActualCartesianPositionLimitsViolation:
            return "kActualCartesianPositionLimitsViolation";
        case Error::kActualCartesianMotionGeneratorElbowLimitViolation:
            return "kActualCartesianMotionGeneratorElbowLimitViolation";
        case Error::kActualJointVelocityLimitsViolation:
            return "kActualJointVelocityLimitsViolation";
        case Error::kActualCartesianVelocityLimitsViolation:
            return "kActualCartesianVelocityLimitsViolation";
        case Error::kActualJointAccelerationLimitsViolation:
            return "kActualJointAccelerationLimitsViolation";
        case Error::kActualCartesianAccelerationLimitsViolation:
            return "kActualCartesianAccelerationLimitsViolation";
        case Error::kCommandJointPositionLimitsViolation:
            return "kCommandJointPositionLimitsViolation";
        case Error::kCommandCartesianPositionLimitsViolation:
            return "kCommandCartesianPositionLimitsViolation";
        case Error::kCommandCartesianMotionGeneratorElbowLimitViolation:
            return "kCommandCartesianMotionGeneratorElbowLimitViolation";
        case Error::kCommandJointVelocityLimitsViolation:
            return "kCommandJointVelocityLimitsViolation";
        case Error::kCommandCartesianVelocityLimitsViolation:
            return "kCommandCartesianVelocityLimitsViolation";
        case Error::kCommandJointAccelerationLimitsViolation:
            return "kCommandJointAccelerationLimitsViolation";
        case Error::kCommandCartesianAccelerationLimitsViolation:
            return "kCommandCartesianAccelerationLimitsViolation";
        case Error::kCommandJointAccelerationDiscontinuity:
            return "kCommandJointAccelerationDiscontinuity";
        case Error::kCollision:
            return "kCollision";
        case Error::kCartesianPositionMotionGeneratorInvalidFrame:
            return "kCartesianPositionMotionGeneratorInvalidFrame";
        case Error::kCommandTorqueDiscontinuity:
            return "kCommandTorqueDiscontinuity";
        case Error::kCommandTorqueRangeViolation:
            return "kCommandTorqueRangeViolation";
        case Error::kInstabilityDetection:
            return "kInstabilityDetection";
    }
    throw std::logic_error("Invalid Error given.");
}

}  // namespace robot
}  // namespace RCI
