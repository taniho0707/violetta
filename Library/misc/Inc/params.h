//******************************************************************************
// @addtogroup MISC
// @file       params.h
// @brief      Parameter Management
//******************************************************************************
#pragma once

#include "stdint.h"
#include "util.h"

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

constexpr uint8_t SLALOM_PARAMS_VELOCITY_LENGTH = 3;  // 0.3, 0.5, 0.7

// 保存先の種類を示す列挙型
enum class ParameterDestinationType : uint8_t {
    HARDCODED = 0,
    INTERNAL_FLASH,
    INTERNAL_RAM,
    EXTERNAL_FRAM,
    CACHE,
};

// スラロームの種類を示す列挙型
// mll::OperationMoveType と同期する必要あり
enum class SlalomType : uint8_t {
    UNDEFINED = 0,
    SLALOM90SML_RIGHT,
    SLALOM90SML_LEFT,
    SLALOM90_RIGHT,
    SLALOM90_LEFT,
    SLALOM180_RIGHT,
    SLALOM180_LEFT,
    SLALOM45IN_RIGHT,
    SLALOM45IN_LEFT,
    SLALOM45OUT_RIGHT,
    SLALOM45OUT_LEFT,
    SLALOM135IN_RIGHT,
    SLALOM135IN_LEFT,
    SLALOM135OUT_RIGHT,
    SLALOM135OUT_LEFT,
    SLALOM90OBL_RIGHT,
    SLALOM90OBL_LEFT,
    LENGTH
};

struct SlalomTransition {
    float x;
    float y;
    float angle;
};

struct SlalomParams {
    float d_before;
    float d_after;
    float acc_rad;
    float max_v_rad;
    float deg;
    float const_deg;
    float in_vel;
    float out_vel;
    float min_vel;
    float acc_lin;
} __attribute__((__packed__));

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
    float battery_ratio;     // バッテリー電圧比率 (ADC * ratio = voltage) [1]

    // Motor Control
    uint32_t motor_control_frequency;    // モータ制御の PWM 周波数 [Hz]
    float motor_control_translation_kp;  // モータ制御の直進方向の P ゲイン [1]
    float motor_control_translation_ki;  // モータ制御の直進方向の I ゲイン [1]
    float motor_control_translation_kd;  // モータ制御の直進方向の D ゲイン [1]
    float motor_control_rotation_kp;     // モータ制御の回転方向の P ゲイン [1]
    float motor_control_rotation_ki;     // モータ制御の回転方向の I ゲイン [1]
    float motor_control_rotation_kd;     // モータ制御の回転方向の D ゲイン [1]
    float motor_control_kabe_kp;         // モータ制御の壁制御の P ゲイン [1]

    float complementary_filter_constant;  // 相補フィルタの定数 1.0の場合100%エンコーダ [1] (0.0 ～ 1.0)

    // Wall Sensor
    uint16_t wallsensor_turnon;                  // 壁センサLEDの立ち上がり待ち時間 [ns]
    uint16_t wallsensor_exist_threshold[6];      // 壁センサの壁有無判定閾値 [1]
    uint16_t wallsensor_center[6];               // 区画中央にいるときの壁センサ値 [1]
    uint16_t wallsensor_kabekire_dif_threshold;  // 壁切れ判定する変位のしきい値 [1]

    // Encoder
    uint16_t encoder_resolution;  // エンコーダの分解能 [1/pulse]

    // IMU
    float imu_sensitivity_acceleration_x;  // IMU の加速度センサの感度 [m/s^2 / LSB]
    float imu_sensitivity_acceleration_y;  // IMU の加速度センサの感度 [m/s^2 / LSB]
    float imu_sensitivity_acceleration_z;  // IMU の加速度センサの感度 [m/s^2 / LSB]
    float imu_offset_acceleration_x;       // IMU の加速度センサのオフセット [m/s^2]
    float imu_offset_acceleration_y;       // IMU の加速度センサのオフセット [m/s^2]
    float imu_offset_acceleration_z;       // IMU の加速度センサのオフセット [m/s^2]

    float imu_sensitivity_gyro_yaw;    // IMU の角速度センサの感度 [rad/s / LSB]
    float imu_sensitivity_gyro_pitch;  // IMU の角速度センサの感度 [rad/s / LSB]
    float imu_sensitivity_gyro_roll;   // IMU の角速度センサの感度 [rad/s / LSB]
    float imu_offset_gyro_yaw;         // IMU の角速度センサのオフセット [rad/s]
    float imu_offset_gyro_pitch;       // IMU の角速度センサのオフセット [rad/s]
    float imu_offset_gyro_roll;        // IMU の角速度センサのオフセット [rad/s]

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

    SlalomParams cache_slalom[SLALOM_PARAMS_VELOCITY_LENGTH][static_cast<uint8_t>(SlalomType::LENGTH)];

   public:
    MouseParams* getCachePointer();

    SlalomParams* getCacheSlalomPointer(float velocity);

    // ハーフサイズを基準にベクトルと回転角(degree)を記述
    constexpr static SlalomTransition param_vectors[] = {
        {           0.f,           0.f,               0.f}, // UNDEFINED
        {         45.0f,         45.0f,      misc::PI / 2}, // SLALOM90SML_RIGHT
        {        -45.0f,         45.0f,     -misc::PI / 2}, // SLALOM90SML_LEFT
        {         90.0f,         90.0f,      misc::PI / 2}, // SLALOM90_RIGHT
        {        -90.0f,         90.0f,     -misc::PI / 2}, // SLALOM90_LEFT
        {         90.0f,          0.0f,          misc::PI}, // SLALOM180_RIGHT
        {        -90.0f,          0.0f,         -misc::PI}, // SLALOM180_LEFT
        {         45.0f,         90.0f,      misc::PI / 4}, // SLALOM45IN_RIGHT
        {        -45.0f,         90.0f,     -misc::PI / 4}, // SLALOM45IN_LEFT
        { 31.819805153f, 95.459415460f,      misc::PI / 4}, // SLALOM45OUT_RIGHT
        {-31.819805153f, 95.459415460f,     -misc::PI / 4}, // SLALOM45OUT_LEFT
        {         90.0f,         45.0f,  3 * misc::PI / 4}, // SLALOM135IN_RIGHT
        {        -90.0f,         45.0f, -3 * misc::PI / 4}, // SLALOM135IN_LEFT
        { 95.459415460f, 31.819805153f,  3 * misc::PI / 4}, // SLALOM135OUT_RIGHT
        {-95.459415460f, 31.819805153f, -3 * misc::PI / 4}, // SLALOM135OUT_LEFT
        { 63.639610307f, 63.639610307f,      misc::PI / 2}, // SLALOM90OBL_RIGHT
        {-63.639610307f, 63.639610307f,     -misc::PI / 2}, // SLALOM90OBL_LEFT
    };

    bool load(ParameterDestinationType from);
    bool save(ParameterDestinationType to);

    bool loadSlalom(ParameterDestinationType from);
    bool saveSlalom(ParameterDestinationType to);

    static Params* getInstance();
};

}  // namespace misc
