//******************************************************************************
// @addtogroup ACT
// @file       act_search.cpp
// @brief      Search Activity
//******************************************************************************
#include "act_search.h"

#include "cmd_format.h"
#include "cmd_server.h"
#include "mll_logger.h"
#include "mll_operation_coordinator.h"
#include "mpl_timer.h"

using namespace act;

void SearchActivity::init(ActivityParameters &params) {
    algorithm = params.search_algorithm;
    oneway = params.only_oneway;
}

Status SearchActivity::run() {
    auto cmd_server = cmd::CommandServer::getInstance();
    auto cmd_ui_out = cmd::CommandFormatUiOut{0};
    // auto cmd_ui_in = cmd::CommandFormatUiIn{0};

    auto maze_solver = mll::MazeSolver::getInstance();

    // TODO: モーター制御が止まっていることを確認する
    auto operation_coordinator = mll::OperationCoordinator::getInstance();
    operation_coordinator->resetPosition(mll::MousePhysicalPosition{45.f, 45.f, 0.f});
    operation_coordinator->enableMotorControl();
    mpl::Timer::sleepMs(1000);

    mll::MultiplePosition goals;
    goals.add(7, 7);
    goals.add(7, 8);
    goals.add(8, 7);
    goals.add(8, 8);

    // Logger setting
    auto logger = mll::Logger::getInstance();
    const uint32_t LOG_ADDRESS = 0x20030000;
    constexpr uint16_t ALL_LOG_LENGTH = 0x20000 / sizeof(mll::LogFormatAll);
    auto logconfig = mll::LogConfig{mll::LogType::ALL, mll::LogDestinationType::INTERNAL_RAM, ALL_LOG_LENGTH, LOG_ADDRESS};
    logger->init(logconfig);
    logger->startPeriodic(mll::LogType::ALL, 5);
    // constexpr uint16_t ALL_LOG_LENGTH = 0x20000 / sizeof(mll::LogFormatSearch);
    // auto logconfig = mll::LogConfig{mll::LogType::SEARCH, mll::LogDestinationType::INTERNAL_RAM, ALL_LOG_LENGTH, LOG_ADDRESS};
    // logger->init(logconfig);

    mll::SearchOptions opt = {
        .maxSearchTime = 0,
        .algorithm = algorithm,
        .oneway = oneway,
        .goals = goals,
        .search_completed = false,
        .search_found_goal = false,
        .velocity_trans = 250,
        .velocity_turn = 250,
        // FIXME: 速度パラメータを使用できていない
    };
    operation_coordinator->runSearch(opt);

    mll::OperationCoordinatorResult state;
    while (true) {
        state = operation_coordinator->state();
        if (state != mll::OperationCoordinatorResult::RUNNING_SEARCH) {
            break;
        }

        // footmap を更新
        const auto current_section = operation_coordinator->getCurrentSection();
        maze_solver->updateFootmap(current_section.x, current_section.y);

        mpl::Timer::sleepMs(1);  // FIXME: 将来的に削除したい
    }

    logger->stopPeriodic(mll::LogType::ALL);

    if (state == mll::OperationCoordinatorResult::ERROR_MOTOR_FAILSAFE) {
        cmd_ui_out.type = mll::UiOutputEffect::DETECT_CRASH;
        cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);
        return Status::ERROR;
    }

    mpl::Timer::sleepMs(3000);
    operation_coordinator->disableMotorControl();

    cmd_ui_out.type = mll::UiOutputEffect::SEARCH_COMPLETE;
    cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);

    return Status::SUCCESS;
}

void SearchActivity::finalize(ActivityParameters &params) {
    params.next_activity = Activities::SELECT_NEXT;
}
