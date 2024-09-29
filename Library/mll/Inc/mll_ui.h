//******************************************************************************
// @addtogroup MLL
// @file       mll_ui.h
// @brief      マウスのUIを管理するクラス
//******************************************************************************
#pragma once

#include <cstdint>

#include "cmd_server.h"
#include "mpl_led.h"
#include "mpl_speaker.h"
#include "msg_format_imu.h"
#include "msg_format_wallsensor.h"
#include "msg_server.h"

namespace mll {

const uint16_t LED_ON_TIME_MAX = 0xFFFF;

const float THRESHOLD_COUNT_GYRO_ROLL = 500.f;   // [dps]
const float THRESHOLD_COUNT_GYRO_PITCH = 400.f;  // [dps]
const float THRESHOLD_COUNT_GYRO_YAW = 400.f;    // [dps]
const float THRESHOLD_STABLE_GYRO = 100.f;       // [dps]
const uint16_t THRESHOLD_WALLSENSOR = 200;       // [LSB]

const uint16_t COUNT_GYRO_ROLL = 30;   // [count]
const uint16_t COUNT_GYRO_PITCH = 30;  // [count]
const uint16_t COUNT_GYRO_YAW = 30;    // [count]
const uint16_t COUNT_WALLSENSOR = 10;  // [count]

// 物理的な UI の状態
enum class UiOutputAction : uint8_t {
    LED,      // LED点灯/消灯
    SPEAKER,  // Speaker 鳴動
    LENGTH
};

enum class UiInputAction : uint8_t {
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

// 物理的な UI と抽象的な UI の変換テーブル
struct UiOutputStatusTable {
    // LED
    uint16_t led_numbers;  // LED 番号の合成値
    uint16_t led_time;     // LED 点灯/点滅時間 [ms]、LED_ON_TIME_MAX で時間制限なし、0の場合は消灯
    float led_freq;        // LED 点灯周波数 [Hz]、0の場合は点灯、負数の場合は消灯
    // Speaker
    mpl::MusicTitle speaker_title;  // Speaker 音楽名
} __attribute__((packed));

struct UiInputStatusTable {};

class Ui {
   private:
    Ui();

    // void loadInputStatusTable();  // NOTE: 入力の抽象化が必要になった場合に実装する
    void loadOutputStatusTable();

    [[maybe_unused]]
    UiInputStatusTable input_status_table[static_cast<uint8_t>(UiInputEffect::LENGTH)];

    UiOutputStatusTable output_status_table[static_cast<uint8_t>(UiOutputEffect::LENGTH)];

    bool enabled;

    // UI Input のためのバッファ
    int16_t count_gyro_roll;  // roll 正方向にしきい値を超え続ける場合に+1、負方向にしきい値を超え続ける場合に-1、しきい値を下回ると0
    int16_t count_gyro_pitch;
    int16_t count_gyro_yaw;
    int16_t count_wallsensor_right;
    int16_t count_wallsensor_left;
    uint16_t count_stable;

    cmd::CommandServer* cmd_server;
    msg::MessageServer* msg_server;

    msg::MsgFormatImu imu_data;
    msg::MsgFormatWallsensor wallsensor_data;  // TODO: WallAnalyzer に移行

    mpl::Led* led;
    mpl::Speaker* speaker;

   public:
    void init();

    // UI の動作を開始する
    // 他の制御は止まっている前提
    void start();

    // UI の動作を停止する
    // Input Message の取得を停止するだけ
    void stop();

    // UI_OUT を処理する
    void output(UiOutputEffect effect);

    // UI_IN を処理する
    // ret: effect の長さ
    uint8_t input(UiInputEffect* effect);

    void interruptPeriodic();

    static Ui* getInstance();
};

}  // namespace mll
