//******************************************************************************
// @addtogroup  Message
// @file        msg_format.h
// @brief       Message Template Format
//******************************************************************************

enum class ModuleId : uint8_t { IMU = 0, WALLSENSOR = 1, LENGTH = 2 };

struct MsgFormat {
    ModuleId module_id;
    uint32_t count;
};
