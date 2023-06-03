/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 */
#pragma once

#include "./platform.h"
#include "robot_datas.h"

#ifdef _WIN32
#include <WinSock2.h>
#endif  // _WIN32

#ifdef _linux_
#include <arpa/inet.h>
#endif  //_linux_

#include <array>
#include <cstdint>
#include <cstring>
#include <type_traits>

/**
 * Tcp命令，在运动开始和结束的时候发送和接收
 */

namespace RCI {
namespace robot {

#pragma pack(push, 1)

/*
 * 所有指令类型
 */
enum class Command : uint32_t {
    kConnect,                //连接机器人，不需要用户自己调用
    kSetMotorPower,          //机器人上电或者下电
    kGetMotorState,          //获取机器人上电状态
    kStartMove,              //开始运动：用户指定机器人的控制模式和运动模式
    kStopMove,               //结束运动，RCI停止给机器人发送运动指令
    kSetCartesianLimit,      //设置笛卡尔空间软限位
    kSetJointLimit,          //设置轴空间软限位
    kSetCollisionBehavior,   //设置碰撞检测阈值
    kSetJointImpedance,      //设置关节阻抗系数
    kSetCartesianImpedance,  //设置笛卡尔空间阻抗系数
    kSetCoor,                //设置末端工具坐标系（相对于法兰）
    kSetLoad,                //设置负载参数
    kSetCartImpDesiredTau,   //笛卡尔空间阻抗控制时，设置机器人末端期望力
    kSetFilters,             //设置机器人控制器的滤波系数
    kSetTorqueFilterCutOffFrequency,
    kAutomaticErrorRecovery,  //发生错误后，机器人自动恢复
    kLoadRobotModel,          //加载动力学模型
    kSetFcCoor,               //设置力控坐标系
    kEnableVirtualGuide,      //开启虚拟墙引导
    kSetDO,                   //设置DO信号
    kGetDIState,              //设置DI信号
    kStartDrag,               //开启拖动
    kStopDrag,                //关闭拖动
    kGetSafetyStopState,      //获取机器人急停状态
    kRebootRobot,             //重启机器人
    kInvalid = 0xff
};

enum class SetStatus : uint8_t { kSuccess, kCommandNotPossibleRejected, kInvalidArgumentRejected };
enum class GetStatus : uint8_t { kSuccess, kCommandNotPossibleRejected, kInvalidArgumentRejected };
enum class RobotOperationMode : uint8_t { kManual, kAuto };
enum class MotorPower : uint8_t { kOFF, kON };
enum class DOSIGNAL : uint8_t { DO0_0, DO0_1, DO0_2, DO0_3, DO1_0, DO1_1 };
enum class DISIGNAL : uint8_t { DI0_0, DI0_1, DI0_2, DI0_3, DI1_0, DI1_1 };
enum class FcCoorType : uint8_t { FCFRAME_WORLD, FCFRAME_TOOL, FCFRAME_PATH };
enum class DragType : uint8_t { TRANSLATION_ONLY, ROTATION_ONLY, FREELY };
enum class DragSpace : uint8_t { JOINT_DRAG, CARTESIAN_DRAG };
enum class SafetyStopType : uint8_t { NOSTOP, ESTOP, GSTOP };

template <typename T>
struct TcpMsg {
    uint16_t len;
    uint32_t command_id;
    T m_cmd;

    void HostToNet() {
        len        = htons(len);
        command_id = htonl(command_id);
        m_cmd.HostToNet();
    }

    void NetToHost() {
        len        = ntohs(len);
        command_id = ntohl(command_id);
        m_cmd.NetToHost();
    }

    /*
     * 数据构造函数，传入指令结构体，自动计算指令长度和指令序号
     */
    TcpMsg(T &_cmd) : len(sizeof(_cmd)), m_cmd(_cmd) {
        // command_id = CMDMGR.GetID()
    }
};

struct ConnectRequest {
    Command command;
    uint16_t udp_port;

    void HostToNet() {
        command  = static_cast<Command>(htonl(static_cast<int32_t>(command)));
        udp_port = htons(udp_port);
    }

    void NetToHost() {
        command  = static_cast<Command>(ntohl(static_cast<int32_t>(command)));
        udp_port = ntohs(udp_port);
    }
};

struct ConnectResponse {
    enum class ConnectStatus : uint8_t { kSuccess, kIncompatibleLibraryVersion };
    Command command;
    ConnectStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct GetMotorStateRequest {
    Command command;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct GetMotorStateResponse {
    enum class ConnectStatus : uint8_t { kSuccess, kIncompatibleLibraryVersion };
    Command command;
    MotorPower motor_state;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetMotorPowerRequest {
    Command command;
    MotorPower motor_state;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetMotorPowerResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct StartMoveRequest {
    enum class ControllerMode : uint8_t { kJointPosition, kCartesianPosition, kJointImpedance, kCartesianImpedance, kTorque, kOther };

    enum class MotionGeneratorMode : uint8_t { kIdle, kJointPosition, kCartesianPosition };

    Command command;
    ControllerMode control_mode;
    MotionGeneratorMode motion_mode;

    StartMoveRequest() : command(Command::kStartMove), control_mode(ControllerMode::kJointPosition), motion_mode(MotionGeneratorMode::kJointPosition) {}

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct StartMoveResponse {
    enum class Status : uint8_t {
        kSuccess,
        kMotionStarted,
        kPreempted,
        kCommandNotPossibleRejected,
        kStartAtSingularPoseRejected,
        kInvalidArgumentRejected,
        kReflexAborted,
        kEmergencyAborted,
        kInputErrorAborted,
        kAborted
    };
    Command command;
    Status status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct StopMoveRequest {
    Command command;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct StopMoveResponse {
    enum class Status : uint8_t { kSuccess, kCommandNotPossibleRejected, kEmergencyAborted, kReflexAborted, kAborted };
    Command command;
    Status status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetLoadRequest {
    Command command;
    double m_lord;
    std::array<double, 3> lord_center;
    std::array<double, 9> I_lord;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetLoadResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetSensorUseTypeRequest {
    Command command;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetSensorUseTypeResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCartesianLimitRequest {
    Command command;
    std::array<double, 3> object_world_size;
    std::array<double, 16> object_frame;
    bool object_activation;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCartesianLimitResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetJointLimitRequest {
    Command command;
    std::array<double, 7> upper_joint_limit;
    std::array<double, 7> lower_joint_limit;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetJointLimitResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCollisionBehaviorRequest {
    Command command;
    std::array<double, 7> upper_torque_thresholds_nominal;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCollisionBehaviorResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetJointImpedanceRequest {
    Command command;
    std::array<double, 7> K_theta;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetJointImpedanceResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCartesianImpedanceRequest {
    Command command;
    std::array<double, 6> K_x;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCartesianImpedanceResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCoorRequest {
    Command command;
    std::array<double, 16> flange2tool;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCoorResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCartImpDesiredTauRequest {
    Command command;
    std::array<double, 6> tau_desired;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetCartImpDesiredTauResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetFiltersRequest {
    Command command;
    double joint_position_filter_frequency;
    double cartesian_position_filter_frequency;
    double torque_filter_frequency;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetFiltersResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};
struct SetTorqueFilterCutOffFrequencyRequest {
    Command command;
    double torque_sensor_filter_frequency;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetTorqueFilterCutOffFrequencyResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct AutomaticErrorRecoveryRequest {
    Command command;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct AutomaticErrorRecoveryResponse {
    enum class Status : uint8_t { kSuccess, kCommandNotPossibleRejected, kManualErrorRecoveryRequiredRejected, kReflexAborted, kEmergencyAborted, kAborted };
    Command command;
    Status status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct LoadRobotModelRequest {
    Command command;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct LoadRobotModelResponse {
    enum class LoadStatus : uint8_t { kSuccess, kFailed };

    Command command;
    LoadStatus status;
    std::array<double, 28> dh_param;
    std::array<double, 21> rob_dim;
    std::array<double, 66> dynamic_param_with_friction;
    std::array<double, 45> dynamic_param_without_friction;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }
    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetFcCoorRequest {
    Command command;
    std::array<double, 16> fc_coor;
    FcCoorType fc_type;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetFcCoorResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct EnableVirtualGuideRequest {
    Command command;

    bool enable;
    std::array<double, 6> stiff_virtual;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }
    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct EnableVirtualGuideResponse {
    enum class EnableStatus : uint8_t { kSuccess, kFailed };

    Command command;
    EnableStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetDORequest {
    Command command;
    DOSIGNAL DO_signal;
    bool DO_state;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct SetDOResponse {
    enum class ConnectStatus : uint8_t { kSuccess, kIncompatibleLibraryVersion };
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct GetDIStateRequest {
    Command command;
    DISIGNAL DI_signal;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct GetDIStateResponse {
    Command command;
    GetStatus status;
    bool DI_state;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct RebootRobotRequest {
    Command command;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct RebootRobotResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct StartDragRequest {
    Command command;
    DragSpace drag_space;
    DragType drag_type;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct StartDragResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct StopDragRequest {
    Command command;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct StopDragResponse {
    Command command;
    SetStatus status;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct GetSafetyStopStateRequest {
    Command command;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

struct GetSafetyStopStateResponse {
    enum class ConnectStatus : uint8_t { kSuccess, kIncompatibleLibraryVersion };
    Command command;
    SafetyStopType safety_stop_state;

    void HostToNet() { command = static_cast<Command>(htonl(static_cast<int32_t>(command))); }

    void NetToHost() { command = static_cast<Command>(ntohl(static_cast<int32_t>(command))); }
};

#pragma pack(pop)

}  // namespace robot
}  // namespace RCI
