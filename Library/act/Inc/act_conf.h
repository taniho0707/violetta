//******************************************************************************
// @addtogroup ACT
// @file       act_conf.h
// @brief      Activity に関する定数を定義
//******************************************************************************
#pragma once

#include "stdint.h"

namespace act {

enum class Status : uint8_t {
    SUCCESS = 0,
    NOIMPLEMENT = 254,
    ERROR = 255,
};

enum class Activities : uint8_t {
    NONE = 0x00,             // 何もしない 使わない想定
    SEARCH = 0x01,           // 探索モード
    SHORTRUN = 0x02,         // 最短経路モード
    SELECT_NEXT = 0xD1,      // 次のアクティビティを選択
    PARAMTUNE_MOTOR = 0xE1,  // モーターパラメータ調整
    MODULE_TEST = 0xF1,      // モジュールテスト
    DEBUG = 0xFF,            // デバッグモード
};

class IActivity {
   public:
    virtual void init() = 0;
    virtual act::Status run() = 0;
    virtual void finalize() = 0;
};

}  // namespace act
