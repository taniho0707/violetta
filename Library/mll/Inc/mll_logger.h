//******************************************************************************
// @addtogroup MLL
// @file       mll_logger.h
// @brief      あらゆるログを取るためのクラス
//******************************************************************************
#pragma once

#include "mll_position.h"
#include "mll_wall.h"
#include "stdint.h"

namespace mll {

// Result を定義
enum class LoggerResult : uint8_t {
    SUCCESS = 0,
    NO_DATA = 250,
    INSUFFICIENT_SIZE = 251,
    NO_IMPLEMENT = 252,
    ACCESS_FAILED = 253,
    DESTINATION_FULL = 254,
    ERROR = 255,
};

// このクラスで対応するログの種類を示す列挙型
enum class LogType : uint8_t {
    ALL = 0,
    SEARCH = 1,
    LENGTH
};

struct LogFormatAll {
    uint32_t time;                   // ログを取得した時間
    float motor_left;                // 左モータの速度
    float motor_right;               // 右モータの速度
    float motor_current_left;        // 左モータの電流
    float motor_current_right;       // 右モータの電流
    float motor_suction;             // 吸引モータの速度
    float encoder_left;              // 左エンコーダの読み値
    float encoder_right;             // 右エンコーダの読み値
    uint16_t wallsensor_frontleft;   // 前左壁センサの値
    uint16_t wallsensor_left;        // 左壁センサの値
    uint16_t wallsensor_center;      // 前壁センサの値
    uint16_t wallsensor_right;       // 右壁センサの値
    uint16_t wallsensor_frontright;  // 前右壁センサの値
    float distance_from_center;      // 左右壁センサから計算した中心からの横方向の距離 (左が正)
    float distance_from_front;       // 前壁センサから計算した前壁からの距離
    float angle_from_front;          // 前壁センサから計算した前壁からの角度
    bool kabekire_left;              // 左側の壁切れ発生可否
    bool kabekire_right;             // 右側の壁切れ発生可否
    float target_v_translation;      // 目標直進速度
    float target_v_rotation;         // 目標回転速度
    float current_v_translation;     // 現在の直進速度
    float current_v_rotation;        // 現在の回転速度
    float position_x;                // 現在の x 座標
    float position_y;                // 現在の y 座標
    float position_theta;            // 現在の角度
    float gyro_yaw;                  // ジャイロの yaw 角度
    float acc_x;                     // 加速度センサの x 軸方向の加速度
    float acc_y;                     // 加速度センサの y 軸方向の加速度
    float acc_z;                     // 加速度センサの z 軸方向の加速度
    float battery;                   // バッテリー電圧
} __attribute__((packed));

struct LogFormatSearch {
    uint32_t time;                           // ログを取得した時間
    MouseSectionPosition current_section;    // 現在の区画
    MousePhysicalPosition current_position;  // 現在の物理位置
    Walldata current_walldata;               // 現在の壁情報
} __attribute__((packed));

// ログの保存先の種類を示す列挙型
enum class LogDestinationType : uint8_t {
    INTERNAL_FLASH = 0,
    INTERNAL_RAM,
    EXTERNAL_FRAM,
};

// ログの保存先とフォーマットを指定する構造体
struct LogConfig {
    LogType type;             // ログの種類
    LogDestinationType dest;  // ログの保存先の種類
    uint32_t length;          // ログの長さ
    uint32_t address;         // ログの保存先の先頭アドレス
    // uint16_t interval;        // ログの保存間隔 0=無効、1=1ms周期、3=3ms周期
};

class Logger {
   private:
    Logger();

    static LogConfig config[static_cast<uint8_t>(LogType::LENGTH)];
    static uint32_t address_next[static_cast<uint8_t>(LogType::LENGTH)];

    // 定期的にロギングするかどうか、するなら何秒おきかを指定する配列
    // 0: 自動ロギング無効、1以上の整数: [ms] おきに自動ロギング
    static uint16_t duration[static_cast<uint8_t>(LogType::LENGTH)];

    static uint16_t counter;  // 自動ロギングのカウンタ、== duration のときにロギング

   public:
    // ログ保存のための初期設定
    static LoggerResult init(LogConfig& config);

    // ログを1回分保存
    static LoggerResult save(LogType type, void* data);

    // ログを読み出す
    static LoggerResult read(LogConfig& config, uint32_t ite, void* data);

    // 自動ロギングを開始する
    static LoggerResult startPeriodic(LogType type, uint16_t duration);

    // 自動ロギングを停止する
    static LoggerResult stopPeriodic(LogType type);

    static Logger* getInstance();

    static void interruptPeriodic();
};

}  // namespace mll
