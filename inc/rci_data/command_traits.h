/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <rci_data/command_types.h>

namespace RCI {
namespace robot {

template <typename T>
struct CommandTraits {};

template <>
struct CommandTraits<GetMotorStateRequest> {
    static constexpr const char* kName = "Send Get Motor State Request";
};
template <>
struct CommandTraits<GetMotorStateResponse> {
    static constexpr const char* kName = "Received Motor State Response";
};

template <>
struct CommandTraits<SetMotorPowerRequest> {
    static constexpr const char* kName = "Send Set Motor Power Request";
};
template <>
struct CommandTraits<SetMotorPowerResponse> {
    static constexpr const char* kName = "Received Motor Power Response";
};

template <>
struct CommandTraits<StartMoveRequest> {
    static constexpr const char* kName = "Send StartMove Request";
};
template <>
struct CommandTraits<StartMoveResponse> {
    static constexpr const char* kName = "Received StartMove Response";
};

template <>
struct CommandTraits<StopMoveRequest> {
    static constexpr const char* kName = "Send StopMove Request ";
};
template <>
struct CommandTraits<StopMoveResponse> {
    static constexpr const char* kName = "Received StopMove Response ";
};

template <>
struct CommandTraits<SetCartesianLimitRequest> {
    static constexpr const char* kName = "Send Set Cartesian Limit Request";
};
template <>
struct CommandTraits<SetCartesianLimitResponse> {
    static constexpr const char* kName = "Received Set Cartesian Limit Response";
};

template <>
struct CommandTraits<SetCollisionBehaviorRequest> {
    static constexpr const char* kName = "Send Set Collision Behavior Request";
};
template <>
struct CommandTraits<SetCollisionBehaviorResponse> {
    static constexpr const char* kName = "Received Set Collision Behavior Response";
};

template <>
struct CommandTraits<SetJointImpedanceRequest> {
    static constexpr const char* kName = "Send Set Joint Impedance Request";
};
template <>
struct CommandTraits<SetJointImpedanceResponse> {
    static constexpr const char* kName = "Received Set Joint Impedance Response";
};

template <>
struct CommandTraits<SetCartesianImpedanceRequest> {
    static constexpr const char* kName = "Send Set Cartesian Impedance Request";
};
template <>
struct CommandTraits<SetCartesianImpedanceResponse> {
    static constexpr const char* kName = "Received Set Cartesian Impedance Response";
};

template <>
struct CommandTraits<SetLoadRequest> {
    static constexpr const char* kName = "Send Set Load Request";
};
template <>
struct CommandTraits<SetLoadResponse> {
    static constexpr const char* kName = "Received Set Load Response";
};

template <>
struct CommandTraits<AutomaticErrorRecoveryRequest> {
    static constexpr const char* kName = "Send Automatic Error Recovery Request";
};
template <>
struct CommandTraits<AutomaticErrorRecoveryResponse> {
    static constexpr const char* kName = "Received Automatic Error Recovery Response";
};

template <>
struct CommandTraits<SetCoorRequest> {
    static constexpr const char* kName = "Send Set Coor Request";
};
template <>
struct CommandTraits<SetCoorResponse> {
    static constexpr const char* kName = "Received Set Coor Response";
};

template <>
struct CommandTraits<SetCartImpDesiredTauRequest> {
    static constexpr const char* kName = "Send Set Desired tau Request";
};
template <>
struct CommandTraits<SetCartImpDesiredTauResponse> {
    static constexpr const char* kName = "Received Set Desired tau Response";
};

template <>
struct CommandTraits<SetFiltersRequest> {
    static constexpr const char* kName = "Send Set Filters Request";
};
template <>
struct CommandTraits<SetFiltersResponse> {
    static constexpr const char* kName = "Received Set Filters Response";
};

template <>
struct CommandTraits<SetTorqueFilterCutOffFrequencyRequest> {
    static constexpr const char* kName = "Send Set Torque Filter cutoff frequency Request";
};
template <>
struct CommandTraits<SetTorqueFilterCutOffFrequencyResponse> {
    static constexpr const char* kName = "Received Set Torque Filter cutoff frequency Response";
};

template <>
struct CommandTraits<EnableVirtualGuideRequest> {
    static constexpr const char* kName = "Send Set Enable Virtual Guide frequency Request";
};
template <>
struct CommandTraits<EnableVirtualGuideResponse> {
    static constexpr const char* kName = "Received Set Enable Virtual Guide frequency Response";
};

template <>
struct CommandTraits<LoadRobotModelRequest> {
    static constexpr const char* kName = "Send Load Robot Model Request";
};
template <>
struct CommandTraits<LoadRobotModelResponse> {
    static constexpr const char* kName = "Received Load Robot Model Response";
};

template <>
struct CommandTraits<SetDORequest> {
    static constexpr const char* kName = "Send Set DO  Request";
};
template <>
struct CommandTraits<SetDOResponse> {
    static constexpr const char* kName = "Received Set DO Response";
};

template <>
struct CommandTraits<GetDIStateRequest> {
    static constexpr const char* kName = "Send Get DI State Request";
};
template <>
struct CommandTraits<GetDIStateResponse> {
    static constexpr const char* kName = "Received Get DI State Response";
};

template <>
struct CommandTraits<SetJointLimitRequest> {
    static constexpr const char* kName = "Send Set Joint Limit Request";
};
template <>
struct CommandTraits<SetJointLimitResponse> {
    static constexpr const char* kName = "Received Set Joint Limit Response";
};

template <>
struct CommandTraits<SetFcCoorRequest> {
    static constexpr const char* kName = "Send Set Fc Coor Request";
};
template <>
struct CommandTraits<SetFcCoorResponse> {
    static constexpr const char* kName = "Received Set Fc Coor Response";
};

template <>
struct CommandTraits<RebootRobotRequest> {
    static constexpr const char* kName = "Send Reboot Robot Request";
};
template <>
struct CommandTraits<RebootRobotResponse> {
    static constexpr const char* kName = "Received Reboot Robot Response";
};

template <>
struct CommandTraits<StartDragRequest> {
    static constexpr const char* kName = "Send Start Drag Request";
};
template <>
struct CommandTraits<StartDragResponse> {
    static constexpr const char* kName = "Received Start Drag Response";
};

template <>
struct CommandTraits<StopDragRequest> {
    static constexpr const char* kName = "Send Stop Drag Request";
};
template <>
struct CommandTraits<StopDragResponse> {
    static constexpr const char* kName = "Received Stop Drag Response";
};

}  // namespace robot
}  // namespace RCI
