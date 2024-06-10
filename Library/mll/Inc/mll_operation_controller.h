//******************************************************************************
// @addtogroup MLL
// @file       mll_operation_controller.h
// @brief      マウスの動作を統括するクラス
//******************************************************************************
#pragma once

#include "params.h"

namespace mll {

class OperationController {
   private:
    OperationController();

   public:
    void init();

    void interruptPeriodic();

    static OperationController* getInstance();
};

}  // namespace mll
