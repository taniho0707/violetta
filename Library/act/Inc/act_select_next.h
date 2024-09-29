//******************************************************************************
// @addtogroup ACT
// @file       act_select_next.h
// @brief      SelectNext Activity
//******************************************************************************
#pragma once

#include "act_conf.h"
#include "mll_ui.h"

namespace act {

enum class MODE_PRIME : uint8_t {
    EXPR = 0x0,  // 探索走行
    SHRT,        // 最短走行
    DEBUG,       // 迷路出力、モーターログ出力
    TUNE,        // 宴会芸、直進、ターン、連続ターン動作
    SENSOR,      // 壁センサ値チェック/モジュールテスト
    LAST         // コレより下に定義しない
};

enum class MODE_EXPR : uint8_t {
    GRAPH = 0x0,
    ADACHI,
    GRAPH_ONEWAY,
    ADACHI_ONEWAY,
    LAST  // コレより下に定義しない
};

enum class MODE_SHRT : uint8_t {
    SMALL = 0x0,
    LARGE,
    DIAGO,
    LAST  // コレより下に定義しない
};
// 迷路番号0〜2を任意に呼び出せるように

enum class MODE_DEBUG : uint8_t {
    OUTPUT_MAZE = 0x0,  // 迷路情報を出力
    OUTPUT_LOG,         // ログ情報を出力
    LAST                // コレより下に定義しない
};

enum class MODE_TUNE : uint8_t {
    STAY = 0x0,
    STRAIGHT_6,
    PIVOTTURN,
    SLALOM90SML_LEFT1,
    SLALOM90SML_RIGHT1,
    SLALOM90SML_LEFT2,
    SLALOM90SML_RIGHT2,
    OVERALL_LEFT,
    OVERALL_RIGHT,
    LAST  // コレより下に定義しない
};

enum class MODE_SENSOR : uint8_t {
    CONSOLE = 0x0,  // UART 出力
    LAST            // コレより下に定義しない
};

struct StructMode {
    uint8_t prime;
    uint8_t sub;
    uint8_t number;
};

const static MODE_PRIME first_mode_prime = MODE_PRIME::EXPR;
const static MODE_EXPR first_mode_expr = MODE_EXPR::GRAPH;
const static MODE_SHRT first_mode_shrt = MODE_SHRT::SMALL;
const static MODE_DEBUG first_mode_debug = MODE_DEBUG::OUTPUT_MAZE;
const static MODE_TUNE first_mode_tune = MODE_TUNE::STAY;
const static MODE_SENSOR first_mode_sensor = MODE_SENSOR::CONSOLE;

class SelectNextActivity : public IActivity {
   private:
    StructMode current;

    mll::Ui *ui;
    cmd::CommandServer *cmd_server;

   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
