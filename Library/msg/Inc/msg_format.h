//******************************************************************************
// @addtogroup  Message
// @file        msg_format.h
// @brief       Message Template Format
//******************************************************************************
#pragma once

#include "stdint.h"

namespace msg {

enum class MsgResult : uint8_t {
    SUCCESS = 0,
    NO_MESSAGE = 253,
    NO_MODULE_ID = 254,
    ERROR = 255
};

enum class ModuleId : uint8_t {
    IMU = 0,
    WALLSENSOR = 1,
    ENCODER = 2,
    BATTERY = 3,
    LENGTH = 4
};

class MsgFormat {
   public:
    MsgFormat(ModuleId id);

    virtual void copy(void* target) = 0;
    virtual void update(void* from) = 0;

    ModuleId getModuleId() { return moduleid; }
    uint32_t getCount() { return count; }
    uint32_t getTime() { return time; }

   protected:
    ModuleId moduleid;
    uint32_t count;  // [times]
    uint32_t time;   // [us]
};

}  // namespace msg
