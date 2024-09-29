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
    NONE = 0x00,       // 何もしない 使わない想定
    INITIALIZE,        // 初期化
    SEARCH,            // 探索モード
    SHORTRUN,          // 最短経路モード
    WALLSENSOR_RUN,    // 壁センサ単体モード for LazuliSensor Only
    SELECT_NEXT,       // 次のアクティビティを選択
    WALLSENSOR_CHECK,  // 壁センサチェック
    PARAMTUNE_MOTOR,   // モーターパラメータ調整
    MODULE_TEST,       // モジュールテスト
    ENKAIGEI,          // 演会芸
    DEBUG,             // デバッグモード
    STANDBY,           // 走行前の待機モード
    LENGTH
};

enum class ActivityTransitionMode : uint8_t {
    FULLAUTO = 0,  // 完全自動遷移モード
    SEMIAUTO,      // 半自動遷移モード (クラッシュ発生以降はMANUALに遷移)
    MANUAL,        // 手動遷移モード (毎回モードセレクトに入る)
    DEBUG,         // デバッグモード
};

enum class SearchAlgorithm : uint8_t {
    GRAPH = 0,  // グラフ探索
    ADACHI,     // 足立法
};

enum class ShortcutMethod : uint8_t {
    NONE = 0,  // 小回り走行
    LARGE,     // 大回り走行
    DIAGO,     // 斜め走行
};

enum class DebugLogType : uint8_t {
    MAZE = 0,  // 迷路情報
    LOG,       // ログ情報
};

enum class MotorParameterTuneType : uint8_t {
    STAY = 0,              // 宴会芸
    STRAIGHT,              // 直進
    PIVOTTURN,             // 超信地旋回
    SMALLTURN_SINGLE,      // 90度小回りの単発
    SMALLTURN_CONTINUOUS,  // 90度小回りの連続
    OVERALLTURN,           // 連続した検証
};

struct ActivityParameters {
    Activities current_activity;             // 現在のアクティビティ
    Activities next_activity;                // 次のアクティビティ
    ActivityTransitionMode transition_mode;  // Activity遷移の種類
    bool needStandby;                        // 次のアクティビティまでにActivity::Standbyをはさむかどうか
    bool initialized;                        // 初期化済みかどうか
    bool crashed;                            // クラッシュしたかどうか
    bool position_recognized;                // 現在の位置情報が正確かどうか
    bool search_completed;                   // 探索が完了したかどうか
    float velocity_trans;                    // 速度 [mm/s]
    float velocity_turn;                     // ターン時の直進速度 [mm/s]

    // Activities::SEARCH
    SearchAlgorithm search_algorithm;  // 探索アルゴリズム
    bool only_oneway;                  // 片道で終了するかどうか

    // Activities::SHORTRUN
    ShortcutMethod shortcut_method;  // 最短経路走行の方法

    // Activities::DEBUG
    DebugLogType debug_log_type;  // デバッグログの種類

    // Activities::PARAMTUNE_MOTOR
    MotorParameterTuneType motor_tune_type;  // モーターパラメータ調整の種類
    bool motor_tune_right;                   // 右モーターを調整するかどうか、falseなら左モーター
};

class IActivity {
   public:
    virtual void init(ActivityParameters &params) = 0;  // 前回のActivityからの申し送りを受け取る
    virtual act::Status run() = 0;
    virtual void finalize(ActivityParameters &params) = 0;  // 次のActivityに渡すパラメータを設定する必要がある
};

}  // namespace act
