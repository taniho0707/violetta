//******************************************************************************
// @addtogroup MLL
// @file       mll_motor_controller.h
// @brief      モーターの回転数制御を行うクラス
//******************************************************************************
#pragma once

#include "cmd_server.h"
#include "msg_format_encoder.h"
#include "msg_format_imu.h"
#include "msg_server.h"
#include "params.h"

namespace mll {

class MotorController {
   private:
    MotorController();

    msg::MessageServer* msg_server;
    cmd::CommandServer* cmd_server;

    msg::MsgFormatEncoder msg_encoder;
    msg::MsgFormatImu msg_imu;

    misc::MouseParams* params;

   public:
    void init();

    void interruptPeriodic();

    static MotorController* getInstance();
};

}  // namespace mll
