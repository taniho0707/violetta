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
    // 次の Activity のポインタを返す
    IActivity* nextActivity(Activities);

    IActivity* current;

    ActivityParameters params;

    IActivity* activity[static_cast<uint8_t>(Activities::LENGTH)];

    act::Status selectNextActivity();  // 次の Activity を判断し、params にセットする

   public:
    Manager(Activities first_activity, ActivityTransitionMode transition_mode);

    void run();
};

}  // namespace act
