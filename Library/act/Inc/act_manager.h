//******************************************************************************
// @addtogroup ACT
// @file       act_manager.h
// @brief      Activity の管理および実行、繊維を管理する
//******************************************************************************
#pragma once

#include "act_conf.h"

// Activities
#include "act_debug.h"
#include "act_module_test.h"
#include "act_none.h"
#include "act_paramtune_motor.h"
#include "act_search.h"
#include "act_select_next.h"
#include "act_shortrun.h"

namespace act {

class Manager {
   private:
    // Activity のインスタンスを生成する
    IActivity* createActivity(Activities);

    IActivity* current;

    Activities next_activity;

   public:
    Manager(Activities first_activity);

    void run();
};

}  // namespace act
