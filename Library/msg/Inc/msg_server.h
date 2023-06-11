//******************************************************************************
// @addtogroup  Message
// @file        msg_server.h
// @brief       Message Server
//******************************************************************************
#include "msg_format.h"

enum class MSGSVR_RESULT : uint8_t { SUCCESS = 0, FAILED = 1 };

class MessageServer {
   private:
    MessageServer();

   public:
    MSGSVR_RESULT sendMessage(MsgFormat format);

    MSGSVR_RESULT receiveMessage(ModuleId id, MsgFormat& format);

    MessageServer& getInstance();
};
