//******************************************************************************
// @addtogroup MISC
// @file       params.h
// @brief      Parameter Management
//******************************************************************************
#pragma once

#include "stdint.h"

// ・パラメータの保管、参照
// ・Flash / FRAM からのロード
// ・Flash / FRAM への保存
// ・UART 経由での読み書き
// ・MCU の UID ごとに管理する
// ・マイコン、機体仕様に依存しないパラメータ化
// ・リポジトリには初期値を置く
// ・ロード優先順位が高い順に
// 　実行時書き換え(RAM)、FLASH 書き込み値、バイナリ埋め込み値
// ・搭載デバイス可否

namespace misc {

enum class LoggingMemoryTarget : uint8_t {
    INTERNALFLASH = 0,
    INTERNALRAM,
    FRAM,
};

struct MouseParams {
    // Params Spec
    uint16_t params_version;  // パラメータのバージョン
    uint32_t mcu_uid[4];      // MCU の UID

    // Mechanical Spec
    float length_front;   // 機体のタイヤより前方の長さ [mm]
    float length_back;    // 機体のタイヤより後方の長さ [mm]
    float tread;          // 機体のトレッド幅 [mm]
    float tire_diameter;  // 機体のタイヤ直径 [mm]
    float tire_width;     // 機体のタイヤ幅 [mm]
    float weight;         // 機体の重量 [g]

    // Battery
    float battery_capacity;  // バッテリー容量 [mAh]
    float battery_warning;   // バッテリー低電圧警告電圧 [mV]
    float battery_shutdown;  // バッテリー強制シャットダウン電圧 [mV]
    float battery_ratio;  // バッテリー電圧比率 (ADC * ratio = voltage) [1]

    // Wall Sensor
    uint16_t wallsensor_turnon;  // 壁センサLEDの立ち上がり待ち時間 [ns]
    uint16_t wallsensor_exist_threshold[6];  // 壁センサの壁有無判定閾値 [1]

    // Memory
    // ログ用内蔵フラッシュメモリのサイズ [byte]
    uint32_t logging_size_internalflash;
    uint32_t logging_size_internalram;  // ログ用内蔵RAMのサイズ [byte]
    uint32_t logging_size_fram;         // ログ用FRAMのサイズ [byte]
} __attribute__((__packed__));

class Params {
   private:
    Params();

    MouseParams cache;

   public:
    MouseParams* getCachePointer();

    static Params* getInstance();
};

}  // namespace misc
