//******************************************************************************
// @addtogroup MLL
// @file       mll_logger.cpp
// @brief      あらゆるログを取るためのクラス
//******************************************************************************
#include "mll_logger.h"

#include "mpl_timer.h"
#include "msg_format_encoder.h"
#include "msg_server.h"

using namespace mll;

LogConfig Logger::config[static_cast<uint8_t>(LogType::LENGTH)];
uint32_t Logger::address_next[static_cast<uint8_t>(LogType::LENGTH)];
uint16_t Logger::duration[static_cast<uint8_t>(LogType::LENGTH)];

Logger::Logger() {
}

LoggerResult Logger::init(LogConfig& config) {
    switch (config.dest) {
        case LogDestinationType::INTERNAL_FLASH:
            return LoggerResult::NO_IMPLEMENT;
        case LogDestinationType::INTERNAL_RAM:
            switch (config.type) {
                case LogType::ENCODER:
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
            case LogType::ENCODER:
                break;
            default:
                return LoggerResult::NO_IMPLEMENT;
        }
    } else {
        return LoggerResult::NO_IMPLEMENT;
    }

    if (Logger::address_next[static_cast<uint8_t>(type)] + sizeof(LogFormatEncoder) > Logger::config[static_cast<uint8_t>(type)].address + sizeof(LogFormatEncoder) * Logger::config[static_cast<uint8_t>(type)].length) {
        return LoggerResult::DESTINATION_FULL;
    } else {
        auto log = static_cast<LogFormatEncoder*>(data);
        *reinterpret_cast<LogFormatEncoder*>(Logger::address_next[static_cast<uint8_t>(type)]) = *log;
        Logger::address_next[static_cast<uint8_t>(type)] += sizeof(LogFormatEncoder);
        return LoggerResult::SUCCESS;
    }

    return LoggerResult::ERROR;
}

LoggerResult Logger::read(LogConfig& config, uint32_t ite, void* data) {
    if (config.dest == LogDestinationType::INTERNAL_RAM) {
        switch (config.type) {
            case LogType::ENCODER:
                break;
            default:
                return LoggerResult::NO_IMPLEMENT;
        }
    } else {
        return LoggerResult::NO_IMPLEMENT;
    }

    if (Logger::config[static_cast<uint8_t>(config.type)].address + sizeof(LogFormatEncoder) * ite > Logger::address_next[static_cast<uint8_t>(config.type)]) {
        return LoggerResult::NO_DATA;
    } else {
        *reinterpret_cast<LogFormatEncoder*>(data) = *reinterpret_cast<LogFormatEncoder*>(Logger::config[static_cast<uint8_t>(config.type)].address + sizeof(LogFormatEncoder) * ite);
        return LoggerResult::SUCCESS;
    }

    return LoggerResult::ERROR;
}

LoggerResult Logger::startPeriodic(LogType type, uint16_t duration) {
    Logger::duration[static_cast<uint8_t>(type)] = duration;
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

    // duration 配列を確認し、0以外の要素に対して save を呼び出す
    for (uint8_t i = 0, len = static_cast<uint8_t>(LogType::LENGTH); i < len; i++) {
        if (Logger::duration[i] != 0) {
            LogType type = static_cast<LogType>(i);
            switch (type) {
                case LogType::ENCODER: {
                    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
                    msg_server->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
                    mll::LogFormatEncoder log = {mpl::Timer::getMicroTime(), msg_encoder.left, msg_encoder.right};
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
