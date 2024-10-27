//******************************************************************************
// @addtogroup ACT
// @file       act_search.cpp
// @brief      Search Activity
//******************************************************************************
#include "act_search.h"

#include "cmd_format.h"
#include "cmd_server.h"
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
    operation_coordinator->enableMotorControl();
    mpl::Timer::sleepMs(1000);

    mll::MultiplePosition goals;
    goals.add(2, 12);
    goals.add(3, 12);
    goals.add(2, 13);
    goals.add(3, 13);

    mll::SearchOptions opt = {
        .maxSearchTime = 0,
        .algorithm = algorithm,
        .oneway = oneway,
        .goals = goals,
        .search_completed = false,
        .search_found_goal = false,
        .velocity_trans = 300,
        .velocity_turn = 300,
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
