//******************************************************************************
// @addtogroup MLL
// @file       mll_logger.cpp
// @brief      あらゆるログを取るためのクラス
//******************************************************************************
#include "mll_logger.h"

#include "mpl_timer.h"
#include "msg_format_battery.h"
#include "msg_format_encoder.h"
#include "msg_format_imu.h"
#include "msg_format_localizer.h"
#include "msg_format_motor.h"
#include "msg_format_motor_controller.h"
#include "msg_format_motor_current.h"
#include "msg_format_wall_analyser.h"
#include "msg_format_wallsensor.h"
#include "msg_server.h"

using namespace mll;

LogConfig Logger::config[static_cast<uint8_t>(LogType::LENGTH)];
uint32_t Logger::address_next[static_cast<uint8_t>(LogType::LENGTH)];
uint16_t Logger::duration[static_cast<uint8_t>(LogType::LENGTH)];
uint16_t Logger::counter = 1;

Logger::Logger() {}

LoggerResult Logger::init(LogConfig& config) {
    switch (config.dest) {
        case LogDestinationType::INTERNAL_FLASH:
            return LoggerResult::NO_IMPLEMENT;
        case LogDestinationType::INTERNAL_RAM:
            switch (config.type) {
                case LogType::ALL:
                case LogType::SEARCH:
                    break;
                default:
                    return LoggerResult::NO_IMPLEMENT;
            }
            break;
        case LogDestinationType::EXTERNAL_FRAM:
            return LoggerResult::NO_IMPLEMENT;
        default:
            return LoggerResult::NO_IMPLEMENT;
    }

    Logger::config[static_cast<uint8_t>(config.type)] = config;
    Logger::address_next[static_cast<uint8_t>(config.type)] = config.address;
    return LoggerResult::SUCCESS;
}

LoggerResult Logger::save(LogType type, void* data) {
    if (Logger::config[static_cast<uint8_t>(type)].dest == LogDestinationType::INTERNAL_RAM) {
        switch (type) {
            case LogType::ALL:
                if (Logger::address_next[static_cast<uint8_t>(LogType::ALL)] + sizeof(LogFormatAll) >
                    Logger::config[static_cast<uint8_t>(type)].address + sizeof(LogFormatAll) * Logger::config[static_cast<uint8_t>(type)].length) {
                    return LoggerResult::DESTINATION_FULL;
                } else {
                    auto log = static_cast<LogFormatAll*>(data);
                    *reinterpret_cast<LogFormatAll*>(Logger::address_next[static_cast<uint8_t>(type)]) = *log;
                    Logger::address_next[static_cast<uint8_t>(type)] += sizeof(LogFormatAll);
                    return LoggerResult::SUCCESS;
                }
            case LogType::SEARCH:
                if (Logger::address_next[static_cast<uint8_t>(LogType::SEARCH)] + sizeof(LogFormatSearch) >
                    Logger::config[static_cast<uint8_t>(type)].address +
                        sizeof(LogFormatSearch) * Logger::config[static_cast<uint8_t>(type)].length) {
                    return LoggerResult::DESTINATION_FULL;
                } else {
                    auto log = static_cast<LogFormatSearch*>(data);
                    *reinterpret_cast<LogFormatSearch*>(Logger::address_next[static_cast<uint8_t>(type)]) = *log;
                    Logger::address_next[static_cast<uint8_t>(type)] += sizeof(LogFormatSearch);
                    return LoggerResult::SUCCESS;
                }
            default:
                return LoggerResult::NO_IMPLEMENT;
        }
    } else {
        return LoggerResult::NO_IMPLEMENT;
    }

    return LoggerResult::ERROR;
}

LoggerResult Logger::read(LogConfig& config, uint32_t ite, void* data) {
    if (config.dest == LogDestinationType::INTERNAL_RAM) {
        switch (config.type) {
            case LogType::ALL:
                if (Logger::config[static_cast<uint8_t>(config.type)].address + sizeof(LogFormatAll) * ite >=
                    Logger::address_next[static_cast<uint8_t>(config.type)]) {
                    return LoggerResult::NO_DATA;
                } else {
                    *reinterpret_cast<LogFormatAll*>(data) =
                        *reinterpret_cast<LogFormatAll*>(Logger::config[static_cast<uint8_t>(config.type)].address + sizeof(LogFormatAll) * ite);
                    return LoggerResult::SUCCESS;
                }
            case LogType::SEARCH:
                if (Logger::config[static_cast<uint8_t>(config.type)].address + sizeof(LogFormatSearch) * ite >=
                    Logger::address_next[static_cast<uint8_t>(config.type)]) {
                    return LoggerResult::NO_DATA;
                } else {
                    *reinterpret_cast<LogFormatSearch*>(data) = *reinterpret_cast<LogFormatSearch*>(
                        Logger::config[static_cast<uint8_t>(config.type)].address + sizeof(LogFormatSearch) * ite);
                    return LoggerResult::SUCCESS;
                }
            default:
                return LoggerResult::NO_IMPLEMENT;
        }
    } else {
        return LoggerResult::NO_IMPLEMENT;
    }

    return LoggerResult::ERROR;
}

LoggerResult Logger::startPeriodic(LogType type, uint16_t duration) {
    Logger::duration[static_cast<uint8_t>(type)] = duration;
    Logger::counter = 1;
    return LoggerResult::SUCCESS;
}

LoggerResult Logger::stopPeriodic(LogType type) {
    Logger::duration[static_cast<uint8_t>(type)] = 0;
    return LoggerResult::SUCCESS;
}

Logger* Logger::getInstance() {
    static Logger instance;
    return &instance;
}

void Logger::interruptPeriodic() {
    static auto msg_server = msg::MessageServer::getInstance();
    static auto msg_motor = msg::MsgFormatMotor();
    static auto msg_motorcurrent = msg::MsgFormatMotorCurrent();
    static auto msg_motor_controller = msg::MsgFormatMotorController();
    static auto msg_encoder = msg::MsgFormatEncoder();
    static auto msg_imu = msg::MsgFormatImu();
    static auto msg_wallsensor = msg::MsgFormatWallsensor();
    static auto msg_wallanalyser = msg::MsgFormatWallAnalyser();
    static auto msg_battery = msg::MsgFormatBattery();
    static auto msg_localizer = msg::MsgFormatLocalizer();

    // duration 配列を確認し、0以外の要素に対して save を呼び出す
    for (uint8_t i = 0, len = static_cast<uint8_t>(LogType::LENGTH); i < len; i++) {
        if (Logger::duration[i] != 0) {
            if (Logger::counter < Logger::duration[i]) {
                Logger::counter++;
                continue;
            }
            Logger::counter = 1;

            LogType type = static_cast<LogType>(i);
            switch (type) {
                case LogType::ALL: {
                    mll::LogFormatAll log = {mpl::Timer::getMicroTime()};
                    msg_server->receiveMessage(msg::ModuleId::MOTOR, &msg_motor);
                    msg_server->receiveMessage(msg::ModuleId::MOTORCURRENT, &msg_motorcurrent);
                    msg_server->receiveMessage(msg::ModuleId::MOTORCONTROLLER, &msg_motor_controller);
                    msg_server->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
                    msg_server->receiveMessage(msg::ModuleId::IMU, &msg_imu);
                    msg_server->receiveMessage(msg::ModuleId::WALLSENSOR, &msg_wallsensor);
                    msg_server->receiveMessage(msg::ModuleId::WALLANALYSER, &msg_wallanalyser);
                    msg_server->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
                    msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
                    log.motor_left = msg_motor.duty_l;
                    log.motor_right = msg_motor.duty_r;
                    log.motor_suction = msg_motor.duty_suction;
                    log.motor_current_left = msg_motorcurrent.current_l;
                    log.motor_current_right = msg_motorcurrent.current_r;
                    log.encoder_left = msg_encoder.left;
                    log.encoder_right = msg_encoder.right;
                    log.wallsensor_frontleft = msg_wallsensor.frontleft;
                    log.wallsensor_left = msg_wallsensor.left;
                    log.wallsensor_center = msg_wallsensor.center;
                    log.wallsensor_right = msg_wallsensor.right;
                    log.wallsensor_frontright = msg_wallsensor.frontright;
                    log.distance_from_center = msg_wallanalyser.distance_from_center;
                    log.distance_from_front = msg_wallanalyser.distance_from_front;
                    log.angle_from_front = msg_wallanalyser.angle_from_front;
                    log.kabekire_left = msg_wallanalyser.kabekire_left;
                    log.kabekire_right = msg_wallanalyser.kabekire_right;
                    log.target_v_translation = msg_motor_controller.velocity_translation;
                    log.target_v_rotation = msg_motor_controller.velocity_rotation;
                    log.current_v_translation = msg_localizer.velocity_translation;
                    log.current_v_rotation = msg_localizer.velocity_rotation;
                    log.position_x = msg_localizer.position_x;
                    log.position_y = msg_localizer.position_y;
                    log.position_theta = msg_localizer.position_theta;
                    log.gyro_yaw = msg_imu.gyro_yaw;
                    log.acc_x = msg_imu.acc_x;
                    log.acc_y = msg_imu.acc_y;
                    log.acc_z = msg_imu.acc_z;
                    log.battery = msg_battery.battery;
                    auto result = Logger::save(static_cast<LogType>(i), &log);
                    if (result != LoggerResult::SUCCESS) {
                        // ログ追加の失敗時には自動ログ追加を無効化する
                        Logger::duration[i] = 0;
                    }
                } break;
                case LogType::SEARCH: {
                    mll::LogFormatSearch log = {mpl::Timer::getMicroTime()};
                    msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
                    msg_server->receiveMessage(msg::ModuleId::WALLANALYSER, &msg_wallanalyser);
                    log.current_position = MousePhysicalPosition{msg_localizer.position_x, msg_localizer.position_y, msg_localizer.position_theta};
                    log.current_walldata = msg_wallanalyser.front_wall;
                    auto result = Logger::save(static_cast<LogType>(i), &log);
                    if (result != LoggerResult::SUCCESS) {
                        // ログ追加の失敗時には自動ログ追加を無効化する
                        Logger::duration[i] = 0;
                    }
                } break;
                default:
                    // FIXME: 他の LogType に対しても同様の処理を追加する
                    break;
            }
        }
    }
}
