//******************************************************************************
// @addtogroup ACT
// @file       act_manager.h
// @brief      Activity の管理および実行、繊維を管理する
//******************************************************************************
#include "act_manager.h"

using namespace act;

Manager::Manager(Activities first_activity) {
    next_activity = first_activity;
    current = nullptr;
}

IActivity* Manager::createActivity(Activities) {
    switch (next_activity) {
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
