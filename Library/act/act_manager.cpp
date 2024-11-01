//******************************************************************************
// @addtogroup ACT
// @file       act_manager.h
// @brief      Activity の管理および実行、繊維を管理する
//******************************************************************************
#include "act_manager.h"

#ifndef MOUSE_LAZULI_SENSOR
#include "act_debug.h"
#include "act_enkaigei.h"
#include "act_initialize.h"
#include "act_module_test.h"
#include "act_paramtune_motor.h"
#include "act_search.h"
#include "act_select_next.h"
#include "act_shortrun.h"
#include "act_standby.h"
#include "act_system_identification.h"
#include "act_wallsensor_check.h"
#endif  // ifndef MOUSE_LAZULI_SENSOR

#include "act_none.h"
#include "act_wallsensor_run.h"

using namespace act;

Manager::Manager(Activities first_activity, ActivityTransitionMode transition_mode) {
    current = nullptr;

    params.current_activity = first_activity;
    params.next_activity = first_activity;
    params.transition_mode = transition_mode;
    params.needStandby = false;
    params.initialized = false;
    params.crashed = false;
    params.position_recognized = true;
    params.search_completed = false;
    params.velocity_trans = 300;  // 探索速度初期値
    params.velocity_turn = 300;   // 探索速度初期値

    activity[static_cast<uint8_t>(Activities::NONE)] = new NoneActivity();

#ifndef MOUSE_LAZULI_SENSOR
    activity[static_cast<uint8_t>(Activities::INITIALIZE)] = new InitializeActivity();
    activity[static_cast<uint8_t>(Activities::SEARCH)] = new SearchActivity();
    activity[static_cast<uint8_t>(Activities::SHORTRUN)] = new ShortrunActivity();
    activity[static_cast<uint8_t>(Activities::SELECT_NEXT)] = new SelectNextActivity();
    activity[static_cast<uint8_t>(Activities::PARAMTUNE_MOTOR)] = new ParamtuneMotorActivity();
    activity[static_cast<uint8_t>(Activities::MODULE_TEST)] = new ModuleTestActivity();
    activity[static_cast<uint8_t>(Activities::DEBUG)] = new DebugActivity();
    activity[static_cast<uint8_t>(Activities::WALLSENSOR_CHECK)] = new WallsensorCheckActivity();
    activity[static_cast<uint8_t>(Activities::ENKAIGEI)] = new EnkaigeiActivity();
    activity[static_cast<uint8_t>(Activities::STANDBY)] = new StandbyActivity();
    activity[static_cast<uint8_t>(Activities::SYSTEM_IDENTIFICATION)] = new SystemIdentificationActivity();
#endif  // ifndef MOUSE_LAZULI_SENSOR

#ifdef MOUSE_LAZULI_SENSOR
    activity[static_cast<uint8_t>(Activities::WALLSENSOR_RUN)] = new WallsensorRunActivity();
#endif  // ifdef MOUSE_LAZULI_SENSOR
}

IActivity* Manager::nextActivity(Activities next_activity) {
    switch (next_activity) {
#ifndef MOUSE_LAZULI_SENSOR
        case Activities::INITIALIZE:
            return activity[static_cast<uint8_t>(Activities::INITIALIZE)];
        case Activities::SEARCH:
            return activity[static_cast<uint8_t>(Activities::SEARCH)];
        case Activities::SHORTRUN:
            return activity[static_cast<uint8_t>(Activities::SHORTRUN)];
        case Activities::SELECT_NEXT:
            return activity[static_cast<uint8_t>(Activities::SELECT_NEXT)];
        case Activities::PARAMTUNE_MOTOR:
            return activity[static_cast<uint8_t>(Activities::PARAMTUNE_MOTOR)];
        case Activities::MODULE_TEST:
            return activity[static_cast<uint8_t>(Activities::MODULE_TEST)];
        case Activities::DEBUG:
            return activity[static_cast<uint8_t>(Activities::DEBUG)];
        case Activities::WALLSENSOR_CHECK:
            return activity[static_cast<uint8_t>(Activities::WALLSENSOR_CHECK)];
        case Activities::ENKAIGEI:
            return activity[static_cast<uint8_t>(Activities::ENKAIGEI)];
        case Activities::STANDBY:
            return activity[static_cast<uint8_t>(Activities::STANDBY)];
        case Activities::SYSTEM_IDENTIFICATION:
            return activity[static_cast<uint8_t>(Activities::SYSTEM_IDENTIFICATION)];
#endif  // ifndef MOUSE_LAZULI_SENSOR
        case Activities::WALLSENSOR_RUN:
            return activity[static_cast<uint8_t>(Activities::WALLSENSOR_RUN)];
        default:
            return activity[static_cast<uint8_t>(Activities::NONE)];
    }
}

act::Status Manager::selectNextActivity() {
    switch (params.transition_mode) {
        case ActivityTransitionMode::FULLAUTO:
            break;
        case ActivityTransitionMode::SEMIAUTO:
            break;
        case ActivityTransitionMode::MANUAL:
            break;
        case ActivityTransitionMode::DEBUG:
            break;
        default:
            return Status::ERROR;
    }
    return Status::ERROR;
}

void Manager::run() {
    while (true) {
        if (params.needStandby) {
            // 次のループを StandbyActivity に差し替える
            params.current_activity = Activities::STANDBY;
            params.needStandby = false;
        } else {
            // 次のアクティビティを選択
            params.current_activity = params.next_activity;
        }

        // 1. Activity のポインタを取得
        current = nextActivity(params.current_activity);

        // 2. Activity の初期化
        current->init(params);

        // 3. Activity の実行
        current->run();

        // 4. Activity の終了処理
        current->finalize(params);
    }
}
