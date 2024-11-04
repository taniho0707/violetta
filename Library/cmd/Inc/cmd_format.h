//******************************************************************************
// @addtogroup  Command
// @file        cmd_format.h
// @brief       Command Template Format
//******************************************************************************
#pragma once

#include "mll_ui_effect.h"
#include "stdint.h"

namespace cmd {

// const uint16_t COMMAND_DEBUG_TX_MESSAGE_LENGTH = 256;
const uint16_t COMMAND_DEBUG_TX_MESSAGE_LENGTH = 8704;
const uint16_t COMMAND_DEBUG_RX_MESSAGE_LENGTH = 64;

enum class CmdResult : uint8_t {
    SUCCESS = 0,
    BUFFER_FULL = 252,
    BUFFER_EMPTY = 253,
    INVALID_COMMAND_ID = 254,
    ERROR = 255
};

enum class CommandId : uint8_t {
    UI_OUT = 0,
    UI_IN = 1,
    DEBUG_TX = 2,
    DEBUG_RX = 3,
    OPERATION_DIRECTION = 4,
    OPERATION_MOVE_ARRAY = 5,
    LENGTH = 6
};

//***************************
//  Debug
//***************************

struct CommandFormatDebugTx {
    uint32_t time;  // [us]
    uint16_t len;
    char message[COMMAND_DEBUG_TX_MESSAGE_LENGTH];
};

struct CommandFormatDebugRx {
    uint32_t time;  // [us]
    uint16_t len;
    char message[COMMAND_DEBUG_RX_MESSAGE_LENGTH];
};

//***************************
//  UI
//***************************

// enum class UiOutType : uint8_t { LED = 0, SPEAKER = 1, LENGTH = 2 };

struct CommandFormatUiOut {
    uint32_t time;  // [us]
    mll::UiOutputEffect type;
};

// enum class UiInType : uint8_t {
//     BUTTON = 0,
//     GYRO = 1,
//     ACCEL = 2,
//     WALLSENSOR = 3,
//     ENCODER = 4,
//     LENGTH = 5
// };

struct CommandFormatUiIn {
    uint32_t time;  // [us]
    mll::UiInputEffect type;
};

//***************************
//  Operation
//***************************

enum class OperationDirectionType : uint8_t {
    STOP = 0,
    STAY,
    SEARCH,
    RUN,
    SPECIFIC,
};

struct CommandFormatOperationDirection {
    uint32_t time;  // [us]
    OperationDirectionType type;
    uint8_t params_index;  // 速度パラメータの番号
};

struct CommandFormatOperationMoveArray {
    uint32_t time;  // [us]
    void* move_array;
    uint16_t length;
};

}  // namespace cmd
