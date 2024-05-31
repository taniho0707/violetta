//******************************************************************************
// @addtogroup MLL
// @file       mll_logger.h
// @brief      あらゆるログを取るためのクラス
//******************************************************************************
#pragma once

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
    MOTOR = 0,
    ENCODER = 1,
    LENGTH
};

// LogType::MOTOR のデータ構造
struct LogFormatMotor {
    uint32_t time;  // ログを取得した時間
    float left;     // 左モータの速度
    float right;    // 右モータの速度
};

// LogType::ENCODER のデータ構造
struct LogFormatEncoder {
    uint32_t time;  // ログを取得した時間
    float left;     // 左モータの速度
    float right;    // 右モータの速度
};

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
};

class Logger {
   private:
    Logger();

    static LogConfig config[static_cast<uint8_t>(LogType::LENGTH)];
    static uint32_t address_next[static_cast<uint8_t>(LogType::LENGTH)];

    // 定期的にロギングするかどうか、するなら何秒おきかを指定する配列
    // ただし、今は 1ms おき以外に対応していない
    // 0: 自動ロギング無効、1以上の整数: [ms] おきに自動ロギング
    static uint16_t duration[static_cast<uint8_t>(LogType::LENGTH)];

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
