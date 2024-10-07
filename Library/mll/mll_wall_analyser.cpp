//******************************************************************************
// @addtogroup MLL
// @file       mll_wall_analyser.cpp
// @brief      壁センサ値から関連情報を取得するクラス
//******************************************************************************
#include "mll_wall_analyser.h"

#include "mll_wall.h"
#include "msg_format_wall_analyser.h"
#include "msg_format_wallsensor.h"
#include "msg_server.h"
#include "params.h"

using namespace mll;

WallAnalyser::WallAnalyser() {}

void WallAnalyser::init() {
    msg_server = msg::MessageServer::getInstance();
    params = misc::Params::getInstance()->getCachePointer();
}

void WallAnalyser::incrementSensorBufferIndex() {
    sensor_buffer_index++;
    if (sensor_buffer_index >= WALLANALYSER_BUFFER_LENGTH) {
        sensor_buffer_index = 0;
    }
}

uint8_t WallAnalyser::previousSensorBufferIndex() {
    if (sensor_buffer_index == 0) {
        return WALLANALYSER_BUFFER_LENGTH - 1;
    } else {
        return sensor_buffer_index - 1;
    }
}

void WallAnalyser::interruptPeriodic() {
    // 壁センサ値を取得
    static msg::MsgFormatWallsensor msg_wallsensor;
    msg_server->receiveMessage(msg::ModuleId::WALLSENSOR, &msg_wallsensor);

    // 壁センサ値をバッファに保存
    incrementSensorBufferIndex();
    sensor_buffer_frontleft[sensor_buffer_index] = msg_wallsensor.frontleft;
    sensor_buffer_left[sensor_buffer_index] = msg_wallsensor.left;
#if defined(MOUSE_LAZULI)
    sensor_buffer_center[sensor_buffer_index] = msg_wallsensor.center;
#endif
    sensor_buffer_right[sensor_buffer_index] = msg_wallsensor.right;
    sensor_buffer_frontright[sensor_buffer_index] = msg_wallsensor.frontright;

    // 左右の変位の計算
    int32_t dif_from_center_left = sensor_buffer_left[sensor_buffer_index] - params->wallsensor_exist_threshold[1];
    int32_t dif_from_center_right = sensor_buffer_right[sensor_buffer_index] - params->wallsensor_exist_threshold[2];
#if defined(MOUSE_ZIRCONIA2KAI)
    int32_t distance_from_front = ((sensor_buffer_frontleft[sensor_buffer_index] - params->wallsensor_center[1]) +
                                   (sensor_buffer_frontright[sensor_buffer_index] - params->wallsensor_center[3])) /
                                  2;
#endif
#if defined(MOUSE_LAZULI)
    int32_t distance_from_front = sensor_buffer_center[sensor_buffer_index] - params->wallsensor_exist_threshold[2];
#endif

    // 壁切れの判定用変数
    bool kabekire_left = false;
    bool kabekire_right = false;
    uint8_t previous_index = previousSensorBufferIndex();

    // 壁有無の判定
    Walldata wall;
    if (sensor_buffer_left[sensor_buffer_index] > params->wallsensor_exist_threshold[1]) {
        // 左壁あり
        wall.addWall(FirstPersonDirection::LEFT);
    } else {
        if (sensor_buffer_left[previous_index] > params->wallsensor_exist_threshold[1] &&
            sensor_buffer_left[previous_index] - sensor_buffer_left[sensor_buffer_index] > params->wallsensor_kabekire_dif_threshold) {
            kabekire_left = true;
        }
        // 壁がなければ壁制御量を 0 にする
        dif_from_center_left = 0;
        // dif_from_center_right *= 2;
    }
    if (sensor_buffer_right[sensor_buffer_index] > params->wallsensor_exist_threshold[2]) {
        // 右壁あり
        wall.addWall(FirstPersonDirection::RIGHT);
    } else {
        if (sensor_buffer_right[previous_index] > params->wallsensor_exist_threshold[2] &&
            sensor_buffer_right[previous_index] - sensor_buffer_right[sensor_buffer_index] > params->wallsensor_kabekire_dif_threshold) {
            kabekire_right = true;
        }
        // 壁がなければ壁制御量を 0 にする
        dif_from_center_right = 0;
        // dif_from_center_left *= 2;
    }

#if defined(MOUSE_ZIRCONIA2KAI)
    if (sensor_buffer_frontleft[sensor_buffer_index] > params->wallsensor_exist_threshold[0] ||
        sensor_buffer_frontright[sensor_buffer_index] > params->wallsensor_exist_threshold[3]) {
        // 前壁あり
        wall.addWall(FirstPersonDirection::FRONT);
    }
#endif
#if defined(MOUSE_LAZULI)
    if (sensor_buffer_center[sensor_buffer_index] > params->wallsensor_exist_threshold[2]) {
        // 前壁あり
        wall.addWall(FirstPersonDirection::FRONT);
    }
#endif

    // FIXME: うまいこといかない
    // 壁制御の有効無効切り替え
    // 変化量が負 (遠ざかる・切れる方向) であれば壁制御量を 0 にする
    if (sensor_buffer_left[sensor_buffer_index] - sensor_buffer_left[previous_index] < 0) {
        dif_from_center_left = 0;
        // dif_from_center_right *= 2;
    }
    if (sensor_buffer_right[sensor_buffer_index] - sensor_buffer_right[previous_index] < 0) {
        dif_from_center_right = 0;
        // dif_from_center_left *= 2;
    }

    // 左右の変位を合成
    int16_t dif_distance_from_center = dif_from_center_left - dif_from_center_right;  // 左寄りが正

    // 壁センサ解析値を格納
    static msg::MsgFormatWallAnalyser msg_wall_analyser;
    msg_wall_analyser.front_wall = wall;
    msg_wall_analyser.distance_from_center = dif_distance_from_center;
    msg_wall_analyser.distance_from_front = distance_from_front;
    msg_wall_analyser.kabekire_left = kabekire_left;
    msg_wall_analyser.kabekire_right = kabekire_right;

    // 解析結果を送信
    msg_server->sendMessage(msg::ModuleId::WALLANALYSER, &msg_wall_analyser);
}

WallAnalyser* WallAnalyser::getInstance() {
    static WallAnalyser instance;
    return &instance;
}
