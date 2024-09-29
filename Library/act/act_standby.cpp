//******************************************************************************
// @addtogroup ACT
// @file       act_standby.cpp
// @brief      Standby Activity
//******************************************************************************
#include "act_standby.h"

#include "cmd_server.h"
#include "mll_operation_coordinator.h"
#include "mpl_speaker.h"
#include "mpl_timer.h"
#include "msg_format_imu.h"
#include "msg_format_localizer.h"
#include "msg_server.h"
#include "params.h"

using namespace act;

void StandbyActivity::init(ActivityParameters &params) {}

Status StandbyActivity::run() {
    auto cmd_server = cmd::CommandServer::getInstance();

    [[maybe_unused]]
    auto operation_coordinator = mll::OperationCoordinator::getInstance();

    [[maybe_unused]]
    auto params_cache = misc::Params::getInstance()->getCachePointer();
    misc::Params::getInstance()->load(misc::ParameterDestinationType::HARDCODED);
    misc::Params::getInstance()->loadSlalom(misc::ParameterDestinationType::HARDCODED);

    [[maybe_unused]]
    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatLocalizer msg_localizer = msg::MsgFormatLocalizer();

    // タイマー割り込みの開始
    mpl::Timer::init();

    auto speaker = mpl::Speaker::getInstance();
    speaker->initPort();
    speaker->playToneSync(mpl::MusicTone::A5, 100);
    speaker->playToneAsync(mpl::MusicTone::D6, 200);

    return Status::SUCCESS;
}

void StandbyActivity::finalize(ActivityParameters &params) {}
