//******************************************************************************
// @addtogroup ACT
// @file       act_search.cpp
// @brief      Search Activity
//******************************************************************************
#include "act_search.h"

#include "mll_operation_coordinator.h"
#include "mpl_timer.h"

using namespace act;

void SearchActivity::init(ActivityParameters &params) {
    algorithm = params.search_algorithm;
}

Status SearchActivity::run() {
    auto operation_coordinator = mll::OperationCoordinator::getInstance();
    operation_coordinator->enableMotorControl();
    mpl::Timer::sleepMs(1000);

    mll::SearchOptions opt = {
        .maxSearchTime = 0,
        .algorithm = mll::AlgorithmType::LEFT_HAND,
        .oneway = true,
        .search_completed = false,
        .search_found_goal = false,
        .velocity_trans = 300,
        .velocity_turn = 300,
    };
    operation_coordinator->runSearch(opt);  // FIXME: 今は左手法固定にしている

    while (true);

    return Status::SUCCESS;
}

void SearchActivity::finalize(ActivityParameters &params) {}
