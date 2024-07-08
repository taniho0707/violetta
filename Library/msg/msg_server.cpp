//******************************************************************************
// @addtogroup MSG
// @file       msg_server.cpp
// @brief      Message Server
//******************************************************************************
#include "msg_server.h"

#include "msg_format_battery.h"
#include "msg_format_encoder.h"
#include "msg_format_imu.h"
#include "msg_format_localizer.h"
#include "msg_format_motor.h"
#include "msg_format_motor_controller.h"
#include "msg_format_motor_current.h"
#include "msg_format_wallsensor.h"

using namespace msg;

MessageServer::MessageServer() {
    // Allocate message queue for each module
    messages[static_cast<uint8_t>(ModuleId::IMU)] = new MsgFormatImu();
    messages[static_cast<uint8_t>(ModuleId::WALLSENSOR)] = new MsgFormatWallsensor();
    messages[static_cast<uint8_t>(ModuleId::ENCODER)] = new MsgFormatEncoder();
    messages[static_cast<uint8_t>(ModuleId::BATTERY)] = new MsgFormatBattery();
    messages[static_cast<uint8_t>(ModuleId::LOCALIZER)] = new MsgFormatLocalizer();
    messages[static_cast<uint8_t>(ModuleId::MOTOR)] = new MsgFormatMotor();
    messages[static_cast<uint8_t>(ModuleId::MOTORCONTROLLER)] = new MsgFormatMotorController();
    messages[static_cast<uint8_t>(ModuleId::MOTORCURRENT)] = new MsgFormatMotorCurrent();
}

MsgResult MessageServer::receiveMessage(ModuleId id, void* format) {
    if (id < ModuleId::LENGTH) {
        messages[static_cast<uint8_t>(id)]->copy(format);
        return MsgResult::SUCCESS;
    } else {
        return MsgResult::NO_MODULE_ID;
    }
}

MsgResult MessageServer::sendMessage(ModuleId id, void* format) {
    if (id < ModuleId::LENGTH) {
        messages[static_cast<uint8_t>(id)]->update(format);
        return MsgResult::SUCCESS;
    } else {
        return MsgResult::NO_MODULE_ID;
    }
}

MessageServer* MessageServer::getInstance() {
    static MessageServer instance;
    return &instance;
}
