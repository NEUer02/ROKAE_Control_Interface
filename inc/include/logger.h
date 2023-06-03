/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once

#include <deque>
#include <string>
#include <vector>

#include "log.h"
#include "rci_data/robot_datas.h"

namespace xmate {

class Logger {
   public:
    explicit Logger(size_t log_size);

    void log(const RCI::robot::RobotState& state, const RCI::robot::RobotCommand& command);

    std::vector<xmate::Record> flush();

   private:
    std::vector<RCI::robot::RobotState> states_;
    std::vector<RCI::robot::RobotCommand> commands_;
    size_t ring_front_{0};
    size_t ring_size_{0};

    const size_t log_size_;
};

}  // namespace xmate
