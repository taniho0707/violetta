//******************************************************************************
// @addtogroup MISC
// @file       params.cpp
// @brief      Parameter Management
//******************************************************************************
#include "params.h"

#include "mcu.h"

using namespace misc;

Params::Params() {}

MouseParams* Params::getCachePointer() {
    return &cache;
}

SlalomParams* Params::getCacheSlalomPointer(float velocity) {
    uint8_t index = 0;
    if (velocity == 300) {
        index = 0;
    } else if (velocity == 500) {
        index = 1;
    } else if (velocity == 700) {
        index = 2;
    }
    return cache_slalom[index];
}

bool Params::load(ParameterDestinationType from) {
    switch (from) {
        case ParameterDestinationType::HARDCODED:
#ifdef MOUSE_LAZULI
            uint32_t uid[4];
            getUid(uid);
            cache.params_version = 0;
            cache.mcu_uid[0] = uid[0];
            cache.mcu_uid[1] = uid[1];
            cache.mcu_uid[2] = uid[2];
            cache.mcu_uid[3] = 0;
            cache.length_front = 0;
            cache.length_back = 0;
            cache.tread = 40;
            cache.tire_diameter = 13;
            cache.tire_width = 4;
            cache.weight = 0;
            cache.battery_capacity = 70;
            cache.battery_warning = 3.6 * 2;
            cache.battery_shutdown = 3.2 * 2;
            cache.battery_ratio = 5.7f /*[R/R]*/ * 3.0f /*[V]*/ / 4095.0f /*[12bit]*/;
            cache.motor_control_frequency = 3200;
            cache.motor_control_translation_kp = 5.f;
            cache.motor_control_translation_ki = 0.9f;
            cache.motor_control_translation_kd = 0;
            cache.motor_control_rotation_kp = 125.f;  // 75.0f + 25.0f;
            cache.motor_control_rotation_ki = 10.f;   // 5.5f + 5.f;
            cache.motor_control_rotation_kd = 0;
            cache.motor_control_kabe_kp = 0.02;
            cache.complementary_filter_constant = 0.5f;
            cache.wallsensor_turnon = 10000;
            cache.wallsensor_exist_threshold[0] = 0;   // TODO: ここの番号とセンサ番号をいい感じに一致させたい
            cache.wallsensor_exist_threshold[1] = 30;  // LEFT
            cache.wallsensor_exist_threshold[2] = 60;  // CENTER
            cache.wallsensor_exist_threshold[3] = 30;  // RIGHT
            cache.wallsensor_exist_threshold[4] = 0;   // FRONTR
            cache.wallsensor_exist_threshold[5] = 0;
            cache.wallsensor_center[0] = 0;    // FRONTL
            cache.wallsensor_center[1] = 115;  // LEFT
            cache.wallsensor_center[2] = 750;  // CENTER
            cache.wallsensor_center[3] = 115;  // RIGHT
            cache.wallsensor_center[4] = 0;    // FRONTR
            cache.wallsensor_center[5] = 0;
            cache.wallsensor_kabekire_dif_threshold = 10;
            cache.encoder_resolution = 160;
            cache.imu_sensitivity_acceleration_x = 2.39f;  // 0.244 mg/LSB
            cache.imu_sensitivity_acceleration_y = 2.39f;  // 0.244 mg/LSB = 2.39 mm/s^2/LSB
            cache.imu_sensitivity_acceleration_z = 2.39f;  // 0.244 mg/LSB
            cache.imu_offset_acceleration_x = 17.f;        // 横方向
            cache.imu_offset_acceleration_y = 55.f;        // 前方向
            cache.imu_offset_acceleration_z = -160.f;      // 下方向
            cache.imu_sensitivity_gyro_yaw = 0.140;        // 140 mdps/LSB
            cache.imu_sensitivity_gyro_pitch = 0.140;      // 140 mdps/LSB
            cache.imu_sensitivity_gyro_roll = 0.140;       // 140 mdps/LSB
            cache.imu_offset_gyro_yaw = 0;
            cache.imu_offset_gyro_pitch = 0;
            cache.imu_offset_gyro_roll = 0;
            cache.logging_size_internalflash = 0;
            cache.logging_size_internalram = 49152;
            cache.logging_size_fram = 0;
            return true;
#endif  // MOUSE_LAZULI
#ifdef MOUSE_ZIRCONIA2KAI
            uint32_t uid[4];
            getUid(uid);
            cache.params_version = 0;
            cache.mcu_uid[0] = uid[0];
            cache.mcu_uid[1] = uid[1];
            cache.mcu_uid[2] = uid[2];
            cache.mcu_uid[3] = 0;
            cache.length_front = 0;
            cache.length_back = 0;
            cache.tread = 40;
            cache.tire_diameter = 13;
            cache.tire_width = 4;
            cache.weight = 0;
            cache.battery_capacity = 40;
            cache.battery_warning = 3.6;
            cache.battery_shutdown = 3.2;
            cache.battery_ratio = 2.0f /*[R/R]*/ * 3.0f /*[V]*/ / 4095.0f /*[12bit]*/;
            cache.motor_control_frequency = 3200;
            cache.motor_control_translation_kp = 0.14902;
            cache.motor_control_translation_ki = 0.10967;
            cache.motor_control_translation_kd = 0;
            cache.motor_control_rotation_kp = 75.f;  // 50.0f;
            cache.motor_control_rotation_ki = 5.5;   // 0.8;
            cache.motor_control_rotation_kd = 0;
            cache.motor_control_kabe_kp = 0.02;
            cache.wallsensor_turnon = 10000;
            cache.wallsensor_exist_threshold[0] = 20;  // TODO: ここの番号とセンサ番号をいい感じに一致させたい
            cache.wallsensor_exist_threshold[1] = 30;
            cache.wallsensor_exist_threshold[2] = 30;
            cache.wallsensor_exist_threshold[3] = 25;
            cache.wallsensor_exist_threshold[4] = 0;
            cache.wallsensor_exist_threshold[5] = 0;
            cache.wallsensor_center[0] = 205;
            cache.wallsensor_center[1] = 115;
            cache.wallsensor_center[2] = 125;
            cache.wallsensor_center[3] = 230;
            cache.wallsensor_center[4] = 0;
            cache.wallsensor_center[5] = 0;
            cache.wallsensor_kabekire_dif_threshold = 10;
            cache.encoder_resolution = 160;
            cache.imu_sensitivity_acceleration = 244.f;  // 0.244 mg/LSB
            cache.imu_offset_acceleration = 0;
            cache.imu_sensitivity_gyro = 0.140;  // 140 mdps/LSB
            cache.imu_offset_gyro = 0;
            cache.logging_size_internalflash = 0;
            cache.logging_size_internalram = 49152;
            cache.logging_size_fram = 0;
            return true;
#endif  // MOUSE_ZIRCONIA2KAI
            return false;
        case ParameterDestinationType::INTERNAL_FLASH:
            return false;
        case ParameterDestinationType::INTERNAL_RAM:
            return false;
        case ParameterDestinationType::EXTERNAL_FRAM:
            return false;
        case ParameterDestinationType::CACHE:
            return false;
    }
    return false;
}

bool Params::save(ParameterDestinationType to) {
    return false;
}

bool Params::loadSlalom(ParameterDestinationType from) {
    switch (from) {
        case ParameterDestinationType::HARDCODED:
#ifdef MOUSE_ZIRCONIA2KAI
            // 300mm/s
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].d_before = 20;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].d_after = 40;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].acc_rad = 1000;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].max_v_rad = 15;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].deg = misc::PI / 2 + 0.15;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].const_deg = misc::PI / 6;

            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].d_before = 20;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].d_after = 40;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].acc_rad = 1000;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].max_v_rad = 15;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].deg = misc::PI / 2 + 0.15;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].const_deg = misc::PI / 6;
#endif  // MOUSE_ZIRCONIA2KAI
#ifdef MOUSE_LAZULI
            // 300mm/s
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].d_before = 20;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].d_after = 40;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].acc_rad = 1000;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].max_v_rad = 15;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].deg = misc::PI / 2 + 0.15;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_RIGHT)].const_deg = misc::PI / 6;

            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].d_before = 20;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].d_after = 40;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].acc_rad = 1000;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].max_v_rad = 15;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].deg = misc::PI / 2 + 0.15;
            cache_slalom[0][static_cast<uint8_t>(SlalomType::SLALOM90SML_LEFT)].const_deg = misc::PI / 6;
#endif  // MOUSE_LAZULI
            return false;
        case ParameterDestinationType::INTERNAL_FLASH:
            return false;
        case ParameterDestinationType::INTERNAL_RAM:
            return false;
        case ParameterDestinationType::EXTERNAL_FRAM:
            return false;
        case ParameterDestinationType::CACHE:
            return false;
    }
    return false;
}

bool Params::saveSlalom(ParameterDestinationType to) {
    return false;
}

Params* Params::getInstance() {
    static Params instance;
    return &instance;
}
