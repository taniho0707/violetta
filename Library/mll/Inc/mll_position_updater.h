//******************************************************************************
// @addtogroup MLL
// @file       mll_position_updater.h
// @brief      Activity と マウス動作の取次を行うクラス
//******************************************************************************
#pragma once

#include "cmd_server.h"
#include "mll_localizer.h"
#include "mll_position.h"
#include "mll_trajectory.h"
#include "msg_format_wall_analyser.h"
#include "msg_server.h"
#include "params.h"
// #include "msg_server.h"

namespace mll {

const float SLALOM_LEFT = 1.f;
const float SLALOM_RIGHT = -1.f;

enum class PositionUpdaterType : uint8_t {
    TIME,
    LOCALIZER,
};

class PositionUpdater {
   private:
    PositionUpdater();

    // マウスの現在の動作
    OperationMoveType current_move;
    float current_move_distance;
    MouseVelocity current_velocity;
    float current_velocity_translation_end;  // ?
    SlalomState slalom_state;
    float slalom_rightleft;

    // スラローム用の内部パラメータ
    float slalom_start_time;     // ターンを開始した時刻 [ms]
    float slalom_move_distance;  // 直線距離のカウント用 [mm]
    bool slalom_front_control;   // スラロームの前壁制御を有効にするかどうか

    // マウスの目標物理座標
    MousePhysicalPosition target_physical_position;

    // 前回更新時の時刻 [ms]
    uint32_t last_update_time;

    // 一動作開始前の目標物理座標、時刻 [ms]
    MousePhysicalPosition start_physical_position;
    uint32_t start_time;

    // 動作モードに関するパラメータ
    PositionUpdaterType type;
    float limit_x;
    float limit_y;
    CardinalDirection start_dir;

    // msg
    msg::MessageServer* msg_server;
    msg::MsgFormatWallAnalyser msg_wall_analyser;

    // cmd
    cmd::CommandServer* cmd_server;
    cmd::CommandFormatUiOut cmd_ui_out;

    misc::MouseParams* params_cache;
    misc::SlalomParams* slalom_params_cache;

    mll::Trajectory trajectory;

    mll::Localizer* localizer;

    // 一動作開始前の初期化
    void initMove();
    void initSlalom(const float rightleft);
    void runSlalom();

    // 現在速度と現在角速度から次の目標物理座標を計算する
    void updateTargetPosition(const float velocity_translation, const float velocity_rotation);

   public:
    // マウスの目標物理座標を更新する
    // この関数は制御周期ごとに1回呼び出される必要がある
    void update();

    // マウスの目標物理座標を取得する
    const MousePhysicalPosition getTargetPhysicalPosition() const;
    // マウスの目標速度と角速度を取得する
    const MouseVelocity getTargetVelocity() const;

    // マウスの次の壁情報を取得できるタイミングかどうかを返す
    // 制御周期ごとに値を更新している
    bool isNextWallReady();

    // マウスの一動作が完了したかどうかを返す
    // 制御周期ごとに値を更新している
    bool isMoveComplete();

    // マウスの次の一動作を設定する
    // 動作が完了していない場合は、現在の動作を上書きする？
    // NOTE: 引数 slalom_front_control が有効の時、前壁がありそうなら前壁制御をする
    void setNextMove(const OperationMoveType move, const float distance, const bool slalom_front_control_flag = false);
    // Localizer による現在位置基準で各一動作の終了判定を行う
    // limit_x / limit_y が負数の場合は無効な値とする
    void setNextMove(const OperationMoveType move, const float distance, const CardinalDirection dir, const float limit_x, const float limit_y,
                     const bool slalom_front_control_flag = false);

    // パラメータをリセットし、目標物理座標を強制的に上書きする
    void reset(const MousePhysicalPosition& position);
    // パラメータをリセットし、目標速度を強制的に上書きする
    void reset(const MouseVelocity& velocity);

    static PositionUpdater* getInstance();
};

}  // namespace mll
