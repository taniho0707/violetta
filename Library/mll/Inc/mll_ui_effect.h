//******************************************************************************
// @addtogroup MLL
// @file       mll_ui_effect.h
// @brief      マウスのUIの抽象的な状態を定義
//******************************************************************************
#pragma once

#include <cstdint>

namespace mll {

// 抽象的な UI の状態
enum class UiOutputEffect : uint8_t {
    // モード選択、ユーザー操作系
    POWERON = 0,     // 電源オン
    MODE_SELECT1,    // モード選択エフェクト1 (第一段階選択)
    MODE_SELECT2,    // モード選択エフェクト2 (第二段階選択)
    MODE_SELECT3,    // モード選択エフェクト3 (第三段階選択)
    MODE_DESELECT1,  // モード選択エフェクト1 (第一段階選択)
    MODE_DESELECT2,  // モード選択エフェクト2 (第二段階選択)
    MODE_DESELECT3,  // モード選択エフェクト3 (第三段階選択)
    MODE_ENTER1,     // モード決定エフェクト1
    MODE_ENTER2,     // モード決定エフェクト2
    MODE_ENTER3,     // モード決定エフェクト3
    RUNSTART,        // 走行開始エフェクト
    DETECT_CRASH,    // クラッシュ検知エフェクト
    // バッテリー状態表示
    BATTERY_FULL,
    BATTERY_HIGH,
    BATTERY_MIDDLE,
    BATTERY_LOW,
    // 走行中の状態表示
    KABEKIRE_RIGHT,  // 右壁切れエフェクト
    KABEKIRE_LEFT,   // 左壁切れエフェクト
    // 探索状態表示
    SEARCH_COMPLETE,  // 探索完了エフェクト
    LED_OFF,          // LED すべて消灯
    // WallAnalyzer 表示
    WALL_EXIST_LEFT,      // 左壁あり
    WALL_EXIST_RIGHT,     // 右壁あり
    WALL_EXIST_FRONT,     // 前壁あり
    WALL_KABEKIRE_LEFT,   // 左壁切れ検知
    WALL_KABEKIRE_RIGHT,  // 右壁切れ検知
    // END
    LENGTH,
};

enum class UiInputEffect : uint8_t {
    // // ユーザー操作系
    // MODE_1_PLUS = 0,  // モード選択1をプラス
    // MODE_1_MINUS,     // モード選択1をマイナス
    // MODE_2_PLUS,      // モード選択2をプラス
    // MODE_2_MINUS,     // モード選択2をマイナス
    // MODE_3_PLUS,      // モード選択3をプラス
    // MODE_3_MINUS,     // モード選択3をマイナス
    // MODE_ENTER,       // モード決定
    // // END
    // LENGTH

    GYRO_ROLL_PLUS,
    GYRO_ROLL_MINUS,
    GYRO_PITCH_PLUS,
    GYRO_PITCH_MINUS,
    GYRO_YAW_PLUS,
    GYRO_YAW_MINUS,
    WALLSENSOR_RIGHT,  // 右壁センサが反応
    WALLSENSOR_LEFT,   // 左壁センサが反応
    STABLE_1SEC,       // 1秒間静止状態が継続
    STABLE_3SEC,       // 3秒間静止状態が継続
    LENGTH
};

}  // namespace mll
