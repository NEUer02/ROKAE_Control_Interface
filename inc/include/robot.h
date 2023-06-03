/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology
 * Co., Ltd, And may contains trade secrets that must be stored and viewed
 * confidentially.
 */
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "control_types.h"
#include "rci_data/command_traits.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"

namespace xmate {

class RobotImpl;

enum class XmateType {
    XMATE3_PRO,  // xmate 7轴 3kg
    XMATE7_PRO,  // xmate 7轴 7kg
    XMATE3,      // xmate 6轴 3kg
    XMATE7       // xmate 6轴 7kg
};

enum class RobotRunningState { UNCONNECTED, CONNECTED, MOVING, DRAGING, ERROR };
class Robot {
    using JointControl     = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
    using TorqueControl    = std::function<Torques(RCI::robot::RobotState robot_state)>;
    using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

   public:
    explicit Robot(const std::string &ip, uint16_t port,const XmateType xmate_type,  bool enforce_realtime = true,size_t log_size = 50);

   public:  // 回调函数，适用于UDP算法交互,用户的控制算法写在这里
    void Control(JointControl joint_control);
    void Control(CartesianControl cartesian_control);
    void Control(TorqueControl torque_control);

   public:  //用户主要调用接口
    /**
     * 设置限幅滤波参数.
     * @param[in] 限幅开启时置为true.
     * @param[in] 截止频率cutoff_frequency，范围是0~1000Hz.建议10~100Hz.
     */
    bool setFilterLimit(bool limit_rate, double cutoff_frequency);

    /**
     * 设置机器人上电状态.
     * @param[in] 机器人上电状态，0代表机器人下电，1代表机器人上电
     */
    bool setMotorPower(int motor_state);

    /**
     * 获取机器人上电状态.
     * @param[in] 机器人上电状态，
     * @return 0代表机器人下电，1代表机器人上电
     */
    int getMotorState();

    /**
     * 设置控制和运动模式.
     *
     * 控制模式有:
     * 1关节位置控制模式.
     * 2笛卡尔空间位置控制模式.
     * 3关节阻抗控制模式.
     * 4笛卡尔空间阻抗控制模式.
     * 5力矩控制模式
     *
     * 运动模式有：
     * 1关节空间轴运动
     * 2笛卡尔空间运动
     * 3空(kIdle)
     *
     * 控制模式与运动模式组合起来进行控制
     * 组合方式提供：
     * 1关节空间轴运动+关节位置控制
     * 2关节空间轴运动+关节阻抗控制模式
     * 3笛卡尔空间运动+笛卡尔空间位置控制模式
     * 4笛卡尔空间运动+笛卡尔空间阻抗控制模式
     * 5力矩控制模式
     *
     * 执行movej,movec,movel指令时不需要调用startmove
     * 在startmove之前应将参数依次设置好，例如滤波阻抗参数等等，设置完成后再
     * 调用startmove指定控制模式和运动模式
     * 在调用startmove后执行其他指令可能会失败，例如下电等操作
     * 正确停止方法是等待回调函数结束，或者调用stopmove
     *
     * 执行MOVEJ MOVEL MOVEC等指令时不需要执行startmove
     * 其他参数设置应该在startmove之前，startmove之后调用函数control开启回调
     * startmove后调用其他接口可能回设置失败，如上下电等
     * 停止运动应该调用stopmove或者等待control函数结束
     *
     * @param[in] ControllerMode.
     * @param[in] MotionGeneratorMode.
     */
    void startMove(RCI::robot::StartMoveRequest::ControllerMode controller_mode, RCI::robot::StartMoveRequest::MotionGeneratorMode motion_generator_mode);

    /**
     * 停止运动，停止下发RobotCommand命令.
     */
    void stopMove();

    /**
     * 接收一次RobotState，不要在运动过程中调用此接口.
     * @return RobotState instance.
     */
    RCI::robot::RobotState receiveRobotState();

    /**
     * 设置碰撞检测阈值.
     * @param[in] 关节碰撞检测阈值 max = {{75, 75, 60, 45, 30, 30, 20}};
     */
    void setCollisionBehavior(const std::array<double, RCI_JOINT_NUM> &upper_torque_thresholds_nominal);

    /**
     * 设置笛卡尔空间运动区域，超过设置区域运动会停止.
     * @param[in] object_world_size 区域长方体长宽高
     * @param[in] object_frame 长方体中心相对于基坐标系位姿
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setCartesianLimit(const std::array<double, 3> &object_world_size, const std::array<double, 16> &object_frame);

    /**
     * 设置关节阻抗控制系数.
     *
     * @param[in] K_theta 关节阻抗系数 ，max={{3000,3000,3000,3000,300,300,300}}
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setJointImpedance(const std::array<double, RCI_JOINT_NUM> &K_theta);

    /**
     * 设置笛卡尔空间阻抗控制系数.
     *
     * @param[in] K_x 笛卡尔空间阻抗控制系数 ,max={{3000,3000,3000,300,300,300}}
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setCartesianImpedance(const std::array<double, 6> &K_x);

    /**
     * 设置末端执行器相对于机器人法兰的位姿.
     *
     * @param[in] F_T_EE 末端执行器坐标系相对于法兰坐标系的转换矩阵
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setCoor(const std::array<double, 16> &F_T_EE);  //

    /**
     * 设置工具和负载的质量、质心和惯性矩阵.
     *
     * @param[in] 工具和负载的质量.
     * @param[in] 工具和负载的质心.
     * @param[in] 工具和负载的惯性矩阵 行优先
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setLoad(const double &load_mass, const std::array<double, 3> &load_center, const std::array<double, 9> &load_inertia);

    /**
     * 设置机器人控制器的滤波截止频率.
     * 截止频率范围在0Hz~1000Hz范围内，1000Hz视为没有进行滤波.建议设置为10~100Hz.
     *
     * @param[in] 关节位置的滤波截止频率.
     * @param[in] 笛卡尔空间位置的滤波截止频率.
     * @param[in] 关节力矩的滤波截止频率.
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     *
     */
    void setFilters(const double &joint_position_filter_frequency, const double &cartesian_position_filter_frequency, const double &torque_filter_frequency);

    /**
     * 笛卡尔空间阻抗时，设置末端期望力.
     *
     * @param[in] 笛卡尔空间末端期望力，desired_tau_max = {{60, 60, 60, 30, 30, 30}}.
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     *
     */
    void setCartImpDesiredTau(const std::array<double, 6> &tau);

    void setTorqueFilterCutOffFrequency(const double &cutoff_frequency);
    /**
     * 当错误发生后，自动恢复机器人.
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void automaticErrorRecovery();

    /**
     * 加载机器人模型参数
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void loadRobotParams(std::array<double, 28> &dh_params, std::array<double, 21> &rob_dims, std::array<double, 66> &dynamic_param_with_friction,
                         std::array<double, 45> &dynamic_param_without_friction);

    void enableVirtualGuide(const bool enable);

    /**
     * 设置机器人IO OUTPUT端信号
     *
     * @param[in] DO_signal DO信号位
     * @param[in] state true or false
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    bool setDO(RCI::robot::DOSIGNAL DO_signal, bool state);

    /**
     * 获取机器人IO INPUT端信号
     * @param[in] DI_signal DI信号位
     * @return true or false
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    bool getDI(RCI::robot::DISIGNAL DI_signal);

    /**
     * 设置机器人关节软限位
     * @param[in] upper_joint_limit 软限位最大值，单位:rad
     * @param[in] lower_joint_limit 软限位最小值, 单位:rad
     * @param[in] auto_limit 置为true时，机器人在即将到达限位时会对关节角度进行限幅处理，机器人不至于下电
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setJointLimit(const std::array<double, RCI_JOINT_NUM> &upper_joint_limit, const std::array<double, RCI_JOINT_NUM> &lower_joint_limit, bool auto_limit);

    /**
     * 设置机器人力控坐标系
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void setFcCoor(const std::array<double, 16> &fc_coor, const RCI::robot::FcCoorType fc_type);

    /**
     * 开启拖动,
     * @param[in] 拖动任务空间，分为笛卡尔空间和轴空间.
     * @param[in] 拖动类型，分为自由拖动，仅平移和仅旋转,轴空间拖动时只有自由拖动.
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void startDrag(const RCI::robot::DragSpace &drag_space, const RCI::robot::DragType &drag_type);

    /**
     * 关闭拖动
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    void stopDrag();

    void rebootRobot();

    /**
     * 获取机器人急停状态，分为安全门急停和急停按钮急停
     *
     * @throw CommandException 控制器返回设置失败.
     * @throw NetworkException 网络连接异常，设置失败.
     */
    RCI::robot::SafetyStopType getSafetyStopState();

    RobotRunningState GetRobotRunningState();

    /**
     * 错误日志写入csv文件
     */
    void writeLogToFile(std::string file_name);

   private:
    JointControl m_joint_control;
    CartesianControl m_cartesian_control;
    TorqueControl m_torque_control;

    RobotImpl *m_robot_impl;
    XmateType m_xmate_type;
    int m_joint_num;
};

}  // namespace xmate
