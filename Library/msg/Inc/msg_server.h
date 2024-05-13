//******************************************************************************
// @addtogroup  Message
// @file        msg_server.h
// @brief       Message Server
//******************************************************************************
#pragma once

#include "msg_format.h"

namespace msg {

class MessageServer {
   private:
    MessageServer();

    MsgFormat* messages[static_cast<uint8_t>(ModuleId::LENGTH)];

   public:
    MsgResult sendMessage(ModuleId id, void* format);

    MsgResult receiveMessage(ModuleId id, void* format);

    static MessageServer* getInstance();
};

}  // namespace msg
