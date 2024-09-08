//******************************************************************************
// @addtogroup ACT
// @file       act_manager.h
// @brief      Activity の管理および実行、繊維を管理する
//******************************************************************************
#include "act_manager.h"

#ifndef MOUSE_LAZULI_SENSOR
#include "act_debug.h"
#include "act_module_test.h"
#include "act_paramtune_motor.h"
#include "act_search.h"
#include "act_select_next.h"
#include "act_shortrun.h"
#include "act_wallsensor_check.h"
#endif  // ifndef MOUSE_LAZULI_SENSOR

#include "act_none.h"
#include "act_wallsensor_run.h"

using namespace act;

Manager::Manager(Activities first_activity) {
    next_activity = first_activity;
    current = nullptr;
}

IActivity* Manager::createActivity(Activities next_activity) {
    switch (next_activity) {
#ifndef MOUSE_LAZULI_SENSOR
        case Activities::SEARCH:
            return new SearchActivity();
        case Activities::SHORTRUN:
            return new ShortrunActivity();
        case Activities::SELECT_NEXT:
            return new SelectNextActivity();
        case Activities::PARAMTUNE_MOTOR:
            return new ParamtuneMotorActivity();
        case Activities::MODULE_TEST:
            return new ModuleTestActivity();
        case Activities::DEBUG:
            return new DebugActivity();
        case Activities::WALLSENSOR_CHECK:
            return new WallsensorCheckActivity();
#endif  // ifndef MOUSE_LAZULI_SENSOR
        case Activities::WALLSENSOR_RUN:
            return new WallsensorRunActivity();
        default:
            return new NoneActivity();
    }
}

void Manager::run() {
    while (true) {
        // 1. Activity のインスタンスを生成する
        current = createActivity(next_activity);

        // 2. Activity の初期化
        current->init();

        // 3. Activity の実行
        current->run();

        // 4. Activity の終了処理
        current->finalize();

        // FIXME: 旧Activityの破棄ができているか確認する
    }
}
