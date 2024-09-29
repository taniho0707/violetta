//******************************************************************************
// @addtogroup ACT
// @file       act_select_next.cpp
// @brief      SelectNext Activity
//******************************************************************************
#include "act_select_next.h"

#include "mpl_debug.h"
#include "mpl_timer.h"
#include "msg_format_battery.h"
#include "msg_format_encoder.h"

using namespace act;

void SelectNextActivity::init(ActivityParameters &params) {
    // TODO: params.crashed の場合は原状復帰の処理を行う

    current = {0};

    // TODO: モーター制御が止まっていることを確認する

    ui = mll::Ui::getInstance();
    ui->start();
    cmd_server = cmd::CommandServer::getInstance();
}

Status SelectNextActivity::run() {
    auto cmd_ui_in = cmd::CommandFormatUiIn{0};
    auto cmd_ui_out = cmd::CommandFormatUiOut{0};
    bool select_end = false;

    // for debug
    auto message = msg::MessageServer::getInstance();
    msg::MsgFormatBattery msg_battery = msg::MsgFormatBattery();
    msg::MsgFormatEncoder msg_encoder = msg::MsgFormatEncoder();
    msg::MsgFormatImu msg_imu = msg::MsgFormatImu();
    msg::MsgFormatWallsensor msg_wallsensor = msg::MsgFormatWallsensor();
    cmd::CommandFormatDebugTx cmd_debug_tx = {0};
    auto debug = mpl::Debug::getInstance();
    mpl::TimerStatistics timer_statistics;

    while (true) {
        mpl::Timer::getStatistics(timer_statistics);
        message->receiveMessage(msg::ModuleId::BATTERY, &msg_battery);
        message->receiveMessage(msg::ModuleId::ENCODER, &msg_encoder);
        message->receiveMessage(msg::ModuleId::IMU, &msg_imu);
        message->receiveMessage(msg::ModuleId::WALLSENSOR, &msg_wallsensor);
        cmd_debug_tx.len = debug->format(
            cmd_debug_tx.message,
            "T: %10d B: %1.2f | L: % 6.4f R: % 6.4f | IMU: % 8.2f, % 7.1f, % 7.1f, % 7.1f | WALL: % 4d,% 4d,% 4d,% 4d,% 4d | STAT: "
            "Max.[%2.0f|%2.0f|%2.0f|%2.0f] "
            "Avg.[%2.0f|%2.0f|%2.0f|%2.0f]\n",
            mpl::Timer::getMicroTime(), msg_battery.battery, msg_encoder.left, msg_encoder.right, msg_imu.gyro_yaw, msg_imu.acc_x / 1000,
            msg_imu.acc_y / 1000, msg_imu.acc_z / 1000, msg_wallsensor.frontleft, msg_wallsensor.left, msg_wallsensor.center, msg_wallsensor.right,
            msg_wallsensor.frontright, float(timer_statistics.count1_max) * 100 / hal::TIMER_COUNT_MAX,
            float(timer_statistics.count2_max) * 100 / hal::TIMER_COUNT_MAX, float(timer_statistics.count3_max) * 100 / hal::TIMER_COUNT_MAX,
            float(timer_statistics.count4_max) * 100 / hal::TIMER_COUNT_MAX, float(timer_statistics.count1_avg) * 100 / hal::TIMER_COUNT_MAX,
            float(timer_statistics.count2_avg) * 100 / hal::TIMER_COUNT_MAX, float(timer_statistics.count3_avg) * 100 / hal::TIMER_COUNT_MAX,
            float(timer_statistics.count4_avg) * 100 / hal::TIMER_COUNT_MAX);
        cmd_server->push(cmd::CommandId::DEBUG_TX, &cmd_debug_tx);

        if (cmd_server->length(cmd::CommandId::UI_IN) > 0) {
            cmd_server->pop(cmd::CommandId::UI_IN, &cmd_ui_in);
            switch (cmd_ui_in.type) {
                case mll::UiInputEffect::GYRO_ROLL_PLUS:
                    if (current.prime + 1 != static_cast<uint8_t>(MODE_PRIME::LAST)) {
                        ++current.prime;
                        current.sub = 0;
                        current.number = 0;
                        cmd_ui_out.type = mll::UiOutputEffect::MODE_SELECT1;
                    }
                    break;
                case mll::UiInputEffect::GYRO_ROLL_MINUS:
                    if (current.prime != 0) {
                        --current.prime;
                        current.sub = 0;
                        current.number = 0;
                        cmd_ui_out.type = mll::UiOutputEffect::MODE_DESELECT1;
                    }
                    break;
                case mll::UiInputEffect::GYRO_PITCH_PLUS:
                    if (current.sub + 1 != static_cast<uint8_t>(MODE_EXPR::LAST)) {
                        ++current.sub;
                        current.number = 0;
                        cmd_ui_out.type = mll::UiOutputEffect::MODE_SELECT2;
                    }
                    break;
                case mll::UiInputEffect::GYRO_PITCH_MINUS:
                    if (current.sub != 0) {
                        --current.sub;
                        current.number = 0;
                        cmd_ui_out.type = mll::UiOutputEffect::MODE_DESELECT2;
                    }
                    break;
                case mll::UiInputEffect::GYRO_YAW_PLUS:
                    break;
                case mll::UiInputEffect::GYRO_YAW_MINUS:
                    break;
                case mll::UiInputEffect::WALLSENSOR_RIGHT:
                    break;
                case mll::UiInputEffect::WALLSENSOR_LEFT:
                    break;
                case mll::UiInputEffect::STABLE_1SEC:
                    break;
                case mll::UiInputEffect::STABLE_3SEC:
                    select_end = true;
                    cmd_ui_out.type = mll::UiOutputEffect::MODE_ENTER1;
                    break;
                default:
                    break;
            }
        }

        if (cmd_ui_out.type != mll::UiOutputEffect::POWERON) {  // 0 でない場合とする
            cmd_server->push(cmd::CommandId::UI_OUT, &cmd_ui_out);
            cmd_ui_out.type = mll::UiOutputEffect::POWERON;
            mpl::Timer::sleepMs(300);
        }

        if (select_end) {
            break;
        }

        mpl::Timer::sleepMs(100);
    }
    return Status::SUCCESS;
}

void SelectNextActivity::finalize(ActivityParameters &params) {
    ui->stop();

    switch (current.prime) {
        case static_cast<uint8_t>(MODE_PRIME::EXPR):
            params.next_activity = Activities::SEARCH;
            params.needStandby = true;
            switch (current.sub) {
                case static_cast<uint8_t>(MODE_EXPR::GRAPH):
                    params.search_algorithm = SearchAlgorithm::GRAPH;
                    params.only_oneway = false;
                    break;
                case static_cast<uint8_t>(MODE_EXPR::ADACHI):
                    params.search_algorithm = SearchAlgorithm::ADACHI;
                    params.only_oneway = false;
                    break;
                case static_cast<uint8_t>(MODE_EXPR::GRAPH_ONEWAY):
                    params.search_algorithm = SearchAlgorithm::GRAPH;
                    params.only_oneway = true;
                    break;
                case static_cast<uint8_t>(MODE_EXPR::ADACHI_ONEWAY):
                    params.search_algorithm = SearchAlgorithm::ADACHI;
                    params.only_oneway = true;
                    break;
                default:
                    break;
            }
            break;
        case static_cast<uint8_t>(MODE_PRIME::SHRT):
            params.next_activity = Activities::SHORTRUN;
            params.needStandby = true;
            switch (current.sub) {
                case static_cast<uint8_t>(MODE_SHRT::SMALL):
                    params.shortcut_method = ShortcutMethod::NONE;
                    break;
                case static_cast<uint8_t>(MODE_SHRT::LARGE):
                    params.shortcut_method = ShortcutMethod::LARGE;
                    break;
                case static_cast<uint8_t>(MODE_SHRT::DIAGO):
                    params.shortcut_method = ShortcutMethod::DIAGO;
                    break;
                default:
                    break;
            }
            break;
        case static_cast<uint8_t>(MODE_PRIME::DEBUG):
            params.next_activity = Activities::DEBUG;
            switch (current.sub) {
                case static_cast<uint8_t>(MODE_DEBUG::OUTPUT_MAZE):
                    params.debug_log_type = DebugLogType::MAZE;
                    break;
                case static_cast<uint8_t>(MODE_DEBUG::OUTPUT_LOG):
                    params.debug_log_type = DebugLogType::LOG;
                    break;
                default:
                    break;
            }
            break;
        case static_cast<uint8_t>(MODE_PRIME::TUNE):
            params.next_activity = Activities::PARAMTUNE_MOTOR;
            params.needStandby = true;
            switch (current.sub) {
                case static_cast<uint8_t>(MODE_TUNE::STAY):
                    params.next_activity = Activities::ENKAIGEI;
                    params.needStandby = false;
                    break;
                case static_cast<uint8_t>(MODE_TUNE::STRAIGHT_6):
                    params.motor_tune_type = MotorParameterTuneType::STRAIGHT;
                    break;
                case static_cast<uint8_t>(MODE_TUNE::PIVOTTURN):
                    params.motor_tune_type = MotorParameterTuneType::PIVOTTURN;
                    params.motor_tune_right = true;
                    break;
                case static_cast<uint8_t>(MODE_TUNE::SLALOM90SML_LEFT1):
                    params.motor_tune_type = MotorParameterTuneType::SMALLTURN_SINGLE;
                    params.motor_tune_right = false;
                    break;
                case static_cast<uint8_t>(MODE_TUNE::SLALOM90SML_RIGHT1):
                    params.motor_tune_type = MotorParameterTuneType::SMALLTURN_SINGLE;
                    params.motor_tune_right = true;
                    break;
                case static_cast<uint8_t>(MODE_TUNE::SLALOM90SML_LEFT2):
                    params.motor_tune_type = MotorParameterTuneType::SMALLTURN_CONTINUOUS;
                    params.motor_tune_right = false;
                    break;
                case static_cast<uint8_t>(MODE_TUNE::SLALOM90SML_RIGHT2):
                    params.motor_tune_type = MotorParameterTuneType::SMALLTURN_CONTINUOUS;
                    params.motor_tune_right = true;
                    break;
                case static_cast<uint8_t>(MODE_TUNE::OVERALL_LEFT):
                    params.motor_tune_type = MotorParameterTuneType::OVERALLTURN;
                    params.motor_tune_right = false;
                    break;
                case static_cast<uint8_t>(MODE_TUNE::OVERALL_RIGHT):
                    params.motor_tune_type = MotorParameterTuneType::OVERALLTURN;
                    params.motor_tune_right = true;
                    break;
                default:
                    break;
            }
            break;
        case static_cast<uint8_t>(MODE_PRIME::SENSOR):
            switch (current.sub) {
                case static_cast<uint8_t>(MODE_SENSOR::CONSOLE):
                    params.next_activity = Activities::WALLSENSOR_CHECK;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}
