//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_controller.cpp
// @brief      マウスの動作を統括するクラス
//******************************************************************************
#include "mll_operation_controller.h"

#include "cmd_server.h"
#include "msg_format_localizer.h"
#include "msg_format_motor_controller.h"
#include "msg_server.h"

mll::OperationController::OperationController() {
    current_operation_direction = cmd::OperationDirectionType::STOP;
}

void mll::OperationController::init() {
    params = misc::Params::getInstance()->getCachePointer();
}

void mll::OperationController::interruptPeriodic() {
    static auto msg_server = msg::MessageServer::getInstance();
    static msg::MsgFormatLocalizer msg_format_localizer;
    static msg::MsgFormatMotorController msg_format_motor_controller;

    static auto cmd_server = cmd::CommandServer::getInstance();
    static cmd::CommandFormatOperationDirection cmd_format_operation_direction;
}

mll::OperationController* mll::OperationController::getInstance() {
    static OperationController instance;
    return &instance;
}
