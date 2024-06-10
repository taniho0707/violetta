//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_controller.cpp
// @brief      マウスの動作を統括するクラス
//******************************************************************************
#include "mll_operation_controller.h"

mll::OperationController::OperationController() {
}

void mll::OperationController::init() {
    // params = misc::Params::getInstance()->getCachePointer();
}

void mll::OperationController::interruptPeriodic() {
}

mll::OperationController* mll::OperationController::getInstance() {
    static OperationController instance;
    return &instance;
}
