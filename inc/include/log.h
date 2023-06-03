/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <vector>

#include "control_types.h"
#include "rci_data/robot_datas.h"

/**
 * @file log.h
 * Contains helper types for logging sent commands and received robot states.
 */

namespace xmate {

/**
 * 用于记录运行数据
 */
struct Record {
    RCI::robot::RobotState state;

    RCI::robot::RobotCommand command;
};

std::string logToCSV(const std::vector<Record>& log);

/**
 * 写入csv文件
 */
void writeLogToFile(const std::vector<Record>& log, std::string file_name);
}  // namespace xmate
