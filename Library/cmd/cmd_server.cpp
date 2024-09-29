//******************************************************************************
// @addtogroup CMD
// @file       cmd_server.cpp
// @brief      Command Server
//******************************************************************************
#include "cmd_server.h"

#include "cmd_format.h"

using namespace cmd;

CommandServer::CommandServer() {
    iteNextUiOut = 0;
    iteLastUiOut = 0;

    iteNextUiIn = 0;
    iteLastUiIn = 0;

    iteNextDebugTx = 0;
    iteLastDebugTx = 0;

    iteNextDebugRx = 0;
    iteLastDebugRx = 0;

    iteNextOperationDirection = 0;
    iteLastOperationDirection = 0;

    iteNextOperationMoveArray = 0;
    iteLastOperationMoveArray = 0;
}

uint16_t CommandServer::length(CommandId id) {
    switch (id) {
        case CommandId::UI_OUT:
            return iteNextUiOut - iteLastUiOut;
        case CommandId::UI_IN:
            return iteNextUiIn - iteLastUiIn;
        case CommandId::DEBUG_TX:
            return iteNextDebugTx - iteLastDebugTx;
        case CommandId::DEBUG_RX:
            return iteNextDebugRx - iteLastDebugRx;
        case CommandId::OPERATION_DIRECTION:
            return iteNextOperationDirection - iteLastOperationDirection;
        case CommandId::OPERATION_MOVE_ARRAY:
            return iteNextOperationMoveArray - iteLastOperationMoveArray;
        default:
            return 0;
    }
}

CmdResult CommandServer::push(CommandId id, void* format) {
    if (length(id) > COMMAND_BUFFER_SIZE - 1) {
        // バッファがフル
        return CmdResult::BUFFER_FULL;
    }
    switch (id) {
        case CommandId::UI_OUT:
            bufferUiOut[iteNextUiOut % COMMAND_BUFFER_SIZE] = *(CommandFormatUiOut*)format;
            ++iteNextUiOut;
            return CmdResult::SUCCESS;
        case CommandId::UI_IN:
            bufferUiIn[iteNextUiIn % COMMAND_BUFFER_SIZE] = *(CommandFormatUiIn*)format;
            ++iteNextUiIn;
            return CmdResult::SUCCESS;
        case CommandId::DEBUG_TX:
            bufferDebugTx[iteNextDebugTx % COMMAND_BUFFER_SIZE] = *(CommandFormatDebugTx*)format;
            ++iteNextDebugTx;
            return CmdResult::SUCCESS;
        case CommandId::DEBUG_RX:
            bufferDebugRx[iteNextDebugRx % COMMAND_BUFFER_SIZE] = *(CommandFormatDebugRx*)format;
            ++iteNextDebugRx;
            return CmdResult::SUCCESS;
        case CommandId::OPERATION_DIRECTION:
            bufferOperationDirection[iteNextOperationDirection % COMMAND_BUFFER_SIZE] = *(CommandFormatOperationDirection*)format;
            ++iteNextOperationDirection;
            return CmdResult::SUCCESS;
        case CommandId::OPERATION_MOVE_ARRAY:
            bufferOperationMoveArray[iteNextOperationMoveArray % COMMAND_BUFFER_SIZE] = *(CommandFormatOperationMoveArray*)format;
            ++iteNextOperationMoveArray;
            return CmdResult::SUCCESS;
        default:
            return CmdResult::INVALID_COMMAND_ID;
    }
}

CmdResult CommandServer::pop(CommandId id, void* format) {
    if (length(id) == 0) {
        return CmdResult::BUFFER_EMPTY;
    }
    switch (id) {
        case CommandId::UI_OUT:
            *(CommandFormatUiOut*)format = bufferUiOut[iteLastUiOut % COMMAND_BUFFER_SIZE];
            ++iteLastUiOut;
            return CmdResult::SUCCESS;
        case CommandId::UI_IN:
            *(CommandFormatUiIn*)format = bufferUiIn[iteLastUiIn % COMMAND_BUFFER_SIZE];
            ++iteLastUiIn;
            return CmdResult::SUCCESS;
        case CommandId::DEBUG_TX:
            *(CommandFormatDebugTx*)format = bufferDebugTx[iteLastDebugTx % COMMAND_BUFFER_SIZE];
            ++iteLastDebugTx;
            return CmdResult::SUCCESS;
        case CommandId::DEBUG_RX:
            *(CommandFormatDebugRx*)format = bufferDebugRx[iteLastDebugRx % COMMAND_BUFFER_SIZE];
            ++iteLastDebugRx;
            return CmdResult::SUCCESS;
        case CommandId::OPERATION_DIRECTION:
            *(CommandFormatOperationDirection*)format = bufferOperationDirection[iteLastOperationDirection % COMMAND_BUFFER_SIZE];
            ++iteLastOperationDirection;
            return CmdResult::SUCCESS;
        case CommandId::OPERATION_MOVE_ARRAY:
            *(CommandFormatOperationMoveArray*)format = bufferOperationMoveArray[iteLastOperationMoveArray % COMMAND_BUFFER_SIZE];
            ++iteLastOperationMoveArray;
            return CmdResult::SUCCESS;
        default:
            return CmdResult::INVALID_COMMAND_ID;
    }
}

CommandServer* CommandServer::getInstance() {
    static CommandServer instance;
    return &instance;
}
