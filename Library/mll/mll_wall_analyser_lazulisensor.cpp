//******************************************************************************
// @addtogroup MLL
// @file       mll_wall_analyser_lazulisensor.cpp
// @brief      壁センサ値から関連情報を取得するクラス
//******************************************************************************
#include "mll_wall_analyser_lazulisensor.h"

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
    auto next_sensor_buffer_index = sensor_buffer_index + 1;
    if (next_sensor_buffer_index >= WALLANALYSER_BUFFER_LENGTH) {
        next_sensor_buffer_index = 0;
    }
    sensor_buffer_frontleft[next_sensor_buffer_index] = msg_wallsensor.frontleft;
    sensor_buffer_left[next_sensor_buffer_index] = msg_wallsensor.left;
    sensor_buffer_center[next_sensor_buffer_index] = msg_wallsensor.center;
    sensor_buffer_right[next_sensor_buffer_index] = msg_wallsensor.right;
    sensor_buffer_frontright[next_sensor_buffer_index] = msg_wallsensor.frontright;
    incrementSensorBufferIndex();

    // TODO: 1ms 未満の間隔のデータに対して入れるフィルタを実装

    // // FIXME: うまいこといかない
    // // 壁制御の有効無効切り替え
    // // 変化量が負 (遠ざかる・切れる方向) であれば壁制御量を 0 にする
    // if (sensor_buffer_left[sensor_buffer_index] - sensor_buffer_left[previous_index] < 0) {
    //     dif_from_center_left = 0;
    //     // dif_from_center_right *= 2;
    // }
    // if (sensor_buffer_right[sensor_buffer_index] - sensor_buffer_right[previous_index] < 0) {
    //     dif_from_center_right = 0;
    //     // dif_from_center_left *= 2;
    // }
}

uint16_t WallAnalyser::getNextSensorBufferSingle(hal::WallSensorNumbers sensor_number) {
    switch (sensor_number) {
        case hal::WallSensorNumbers::FRONTLEFT:
            return sensor_buffer_frontleft[sensor_buffer_index];
        case hal::WallSensorNumbers::LEFT:
            return sensor_buffer_left[sensor_buffer_index];
        case hal::WallSensorNumbers::CENTER:
            return sensor_buffer_center[sensor_buffer_index];
        case hal::WallSensorNumbers::RIGHT:
            return sensor_buffer_right[sensor_buffer_index];
        case hal::WallSensorNumbers::FRONTRIGHT:
            return sensor_buffer_frontright[sensor_buffer_index];
        default:
            return 0;
    }
}

WallAnalyser* WallAnalyser::getInstance() {
    static WallAnalyser instance;
    return &instance;
}
