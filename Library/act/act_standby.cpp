//******************************************************************************
// @addtogroup ACT
// @file       act_standby.cpp
// @brief      Standby Activity
//******************************************************************************
#include "act_standby.h"

#include "cmd_format.h"
#include "cmd_server.h"
#include "mpl_timer.h"

using namespace act;

void StandbyActivity::init(ActivityParameters &params) {}

Status StandbyActivity::run() {
    auto cmd_server = cmd::CommandServer::getInstance();
    auto cmd_ui_in = cmd::CommandFormatUiIn{0};
    auto cmd_ui_out = cmd::CommandFormatUiOut{0};

    while (true) {
        if (cmd_server->length(cmd::CommandId::UI_IN) > 0) {
            cmd_server->pop(cmd::CommandId::UI_IN, &cmd_ui_in);
            if (cmd_ui_in.type == mll::UiInputEffect::WALLSENSOR_RIGHT) {
                cmd_ui_out.type = mll::UiOutputEffect::RUNSTART;
                cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);
                break;
            }
        }
        mpl::Timer::sleepMs(100);
    }

    return Status::SUCCESS;
}

void StandbyActivity::finalize(ActivityParameters &params) {}
