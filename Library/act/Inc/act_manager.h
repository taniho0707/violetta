//******************************************************************************
// @addtogroup ACT
// @file       act_manager.h
// @brief      Activity の管理および実行、繊維を管理する
//******************************************************************************
#pragma once

#include "act_conf.h"

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
