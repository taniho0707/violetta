//******************************************************************************
// @addtogroup ACT
// @file       act_debug.cpp
// @brief      Debug Activity
//******************************************************************************
#include "act_enkaigei.h"

#include "cmd_format.h"
#include "cmd_server.h"
#include "mll_operation_coordinator.h"
#include "mpl_timer.h"

using namespace act;

void EnkaigeiActivity::init(ActivityParameters &params) {}

#ifndef MOUSE_LAZULI_SENSOR
Status EnkaigeiActivity::run() {
    auto operation_coordinator = mll::OperationCoordinator::getInstance();
    auto cmd_server = cmd::CommandServer::getInstance();
    auto cmd_ui_out = cmd::CommandFormatUiOut{0};
    auto cmd_ui_in = cmd::CommandFormatUiIn{0};

    // TODO: モーター制御が止まっていることを確認する

    operation_coordinator->enableMotorControl();

    while (true) {
        if (cmd_server->length(cmd::CommandId::UI_IN) > 0) {
            cmd_server->pop(cmd::CommandId::UI_IN, &cmd_ui_in);
            if (cmd_ui_in.type == mll::UiInputEffect::WALLSENSOR_LEFT) {
                break;
            }
        }
        mpl::Timer::sleepMs(100);
    }

    operation_coordinator->disableMotorControl();
    cmd_ui_out.type = mll::UiOutputEffect::SEARCH_COMPLETE;
    cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);

    return Status::SUCCESS;
}
#endif  // ifndef MOUSE_LAZULI_SENSOR

#ifdef MOUSE_LAZULI_SENSOR
Status EnkaigeiActivity::run() {
    return Status::ERROR;
}
#endif  // MOUSE_LAZULI_SENSOR

void EnkaigeiActivity::finalize(ActivityParameters &params) {
    params.next_activity = Activities::SELECT_NEXT;
}
