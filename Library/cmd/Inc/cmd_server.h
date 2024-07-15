//******************************************************************************
// @addtogroup  Command
// @file        cmd_server.h
// @brief       Command Server
//******************************************************************************
#pragma once

#include "cmd_format.h"

namespace cmd {

const uint16_t COMMAND_BUFFER_SIZE = 16;

class CommandServer {
   private:
    CommandServer();

    uint32_t iteNextDebugTx;  // 次に追加する位置
    uint32_t iteLastDebugTx;  // 次に読み出す位置
    CommandFormatDebugTx bufferDebugTx[COMMAND_BUFFER_SIZE];

    uint32_t iteNextDebugRx;
    uint32_t iteLastDebugRx;
    CommandFormatDebugRx bufferDebugRx[COMMAND_BUFFER_SIZE];

    uint32_t iteNextOperationDirection;
    uint32_t iteLastOperationDirection;
    CommandFormatOperationDirection bufferOperationDirection[COMMAND_BUFFER_SIZE];

    uint32_t iteNextOperationMoveArray;
    uint32_t iteLastOperationMoveArray;
    CommandFormatOperationMoveArray bufferOperationMoveArray[COMMAND_BUFFER_SIZE];

   public:
    uint16_t length(CommandId id);

    CmdResult push(CommandId id, void* format);

    CmdResult pop(CommandId id, void* format);

    static CommandServer* getInstance();
};

}  // namespace cmd
