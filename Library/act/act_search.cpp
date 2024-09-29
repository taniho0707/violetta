//******************************************************************************
// @addtogroup ACT
// @file       act_search.cpp
// @brief      Search Activity
//******************************************************************************
#include "act_search.h"

#include "mll_localizer.h"
#include "mll_motor_controller.h"
#include "mll_operation_coordinator.h"
#include "mpl_timer.h"

using namespace act;

void SearchActivity::init(ActivityParameters &params) {}

Status SearchActivity::run() {
    auto motor_controller = mll::MotorController::getInstance();
    motor_controller->init();

    auto localizer = mll::Localizer::getInstance();
    localizer->init();

    auto operation_coordinator = mll::OperationCoordinator::getInstance();

    // OperationController のテスト
    mpl::Timer::sleepMs(3000);
    motor_controller->setStay();
    mpl::Timer::sleepMs(2000);

    operation_coordinator->enableMotorControl();
    operation_coordinator->runSearch(mll::AlgorithmType::LEFT_HAND);

    while (true);

    return Status::ERROR;
}

void SearchActivity::finalize(ActivityParameters &params) {}
