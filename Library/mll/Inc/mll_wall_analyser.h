//******************************************************************************
// @addtogroup MLL
// @file       mll_wall_analyser.h
// @brief      壁センサ値から関連情報を取得するクラス
//******************************************************************************
#pragma once

#include "msg_server.h"
#include "params.h"

namespace mll {

constexpr uint8_t WALLANALYSER_BUFFER_LENGTH = 10;

class WallAnalyser {
   private:
    WallAnalyser();

    msg::MessageServer* msg_server;

    misc::MouseParams* params;

    // 過去の壁センサ値を残すバッファ
    uint16_t sensor_buffer_frontleft[WALLANALYSER_BUFFER_LENGTH];
    uint16_t sensor_buffer_left[WALLANALYSER_BUFFER_LENGTH];
    uint16_t sensor_buffer_center[WALLANALYSER_BUFFER_LENGTH];
    uint16_t sensor_buffer_right[WALLANALYSER_BUFFER_LENGTH];
    uint16_t sensor_buffer_frontright[WALLANALYSER_BUFFER_LENGTH];
    uint8_t sensor_buffer_index;  // 指し示している先が最新、一つ若い数字が一つ前のデータ

    void incrementSensorBufferIndex();
    uint8_t previousSensorBufferIndex();

   public:
    void init();

    void interruptPeriodic();

    static WallAnalyser* getInstance();
};

}  // namespace mll
