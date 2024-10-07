//******************************************************************************
// @addtogroup MLL
// @file       mll_ui.cpp
// @brief      マウスのUIを管理するクラス
//******************************************************************************
#include "mll_ui.h"

#include "hal_conf.h"
#include "util.h"

using namespace mll;

constexpr uint8_t UI_IN_BUF_LENGTH = static_cast<uint16_t>(UiInputAction::LENGTH);

Ui::Ui() {}

// void Ui::loadInputStatusTable() {}

void Ui::loadOutputStatusTable() {
#if defined(MOUSE_LAZULI)
    output_status_table[static_cast<uint8_t>(UiOutputEffect::POWERON)] = {
        // .led_numbers =
        // static_cast<uint16_t>(hal::LedNumbers::MIDDLE1 | hal::LedNumbers::MIDDLE2 | hal::LedNumbers::MIDDLE3 | hal::LedNumbers::MIDDLE4),
        .led_numbers = 0,
        // .led_time = 1000,
        .led_time = 0,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::STARTUP,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_SELECT1)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::SELECT1,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_SELECT2)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::SELECT2,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_SELECT3)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::SELECT3,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_DESELECT1)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::DESELECT1,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_DESELECT2)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::DESELECT2,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_DESELECT3)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::DESELECT3,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_ENTER1)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::ENTER1,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_ENTER2)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::ENTER2,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::MODE_ENTER3)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::ENTER3,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::RUNSTART)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::RUNSTART,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::DETECT_CRASH)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::ERROR,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::BATTERY_FULL)] = {
        .led_numbers =
            static_cast<uint16_t>(hal::LedNumbers::MIDDLE1 | hal::LedNumbers::MIDDLE2 | hal::LedNumbers::MIDDLE3 | hal::LedNumbers::MIDDLE4),
        .led_time = LED_ON_TIME_MAX,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::BATTERY_HIGH)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::MIDDLE2 | hal::LedNumbers::MIDDLE3 | hal::LedNumbers::MIDDLE4),
        .led_time = LED_ON_TIME_MAX,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::BATTERY_MIDDLE)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::MIDDLE3 | hal::LedNumbers::MIDDLE4),
        .led_time = LED_ON_TIME_MAX,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::BATTERY_LOW)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::MIDDLE4),
        .led_time = LED_ON_TIME_MAX,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::KABEKIRE_RIGHT)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::PULSE_HIGH,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::KABEKIRE_LEFT)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::PULSE_LOW,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::SEARCH_COMPLETE)] = {
        .led_numbers = 0,
        .led_time = 1000,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::SEARCHCOMPLETE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::LED_OFF)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::FLAG | hal::LedNumbers::FRONTL | hal::LedNumbers::LEFT | hal::LedNumbers::FRONT |
                                             hal::LedNumbers::RIGHT | hal::LedNumbers::FRONTR | hal::LedNumbers::MIDDLE1 | hal::LedNumbers::MIDDLE2 |
                                             hal::LedNumbers::MIDDLE3 | hal::LedNumbers::MIDDLE4),
        .led_time = 0,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::WALL_EXIST_LEFT)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::LEFT),
        .led_time = 0,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::WALL_EXIST_RIGHT)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::RIGHT),
        .led_time = 0,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::WALL_EXIST_FRONT)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::FRONT),
        .led_time = 0,
        .led_freq = 0,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::WALL_KABEKIRE_LEFT)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::LEFT),
        .led_time = 200,
        .led_freq = 1,
        .speaker_title = mpl::MusicTitle::NONE,
    };
    output_status_table[static_cast<uint8_t>(UiOutputEffect::WALL_KABEKIRE_RIGHT)] = {
        .led_numbers = static_cast<uint16_t>(hal::LedNumbers::RIGHT),
        .led_time = 200,
        .led_freq = 1,
        .speaker_title = mpl::MusicTitle::NONE,
    };
#endif  // MOUSE_LAZULI
}

void Ui::init() {
    enabled = false;
    cmd_server = cmd::CommandServer::getInstance();
    msg_server = msg::MessageServer::getInstance();
    led = mpl::Led::getInstance();
    speaker = mpl::Speaker::getInstance();
    // loadInputStatusTable();
    loadOutputStatusTable();
}

void Ui::start() {
    enabled = true;
}

void Ui::stop() {
    enabled = false;

    count_gyro_roll = 0;
    count_gyro_pitch = 0;
    count_gyro_yaw = 0;
    count_wallsensor_right = 0;
    count_wallsensor_left = 0;
    count_stable = 0;
}

void Ui::output(UiOutputEffect effect) {
    auto status = output_status_table[static_cast<uint8_t>(effect)];

    // LED
    hal::LedNumbers led_numbers = static_cast<hal::LedNumbers>(status.led_numbers);
    uint16_t led_time = status.led_time;
    float led_freq = status.led_freq;
    if (static_cast<uint16_t>(led_numbers) != 0) {
        if (led_time == 0 || led_freq < 0) {
            led->off(led_numbers);
        } else if (led_freq == 0) {
            led->on(led_numbers);
        } else {
            led->flickAsync(led_numbers, led_freq, led_time);
        }
    }

    // Speaker
    mpl::MusicTitle speaker_title = status.speaker_title;
    if (speaker_title != mpl::MusicTitle::NONE) {
        speaker->playMusicAsync(speaker_title);
    }
}

uint8_t Ui::input(UiInputEffect* effect) {
    uint8_t count = 0;
    msg_server->receiveMessage(msg::ModuleId::IMU, &imu_data);
    msg_server->receiveMessage(msg::ModuleId::WALLSENSOR, &wallsensor_data);

    // GYRO
    if (imu_data.gyro_roll > THRESHOLD_COUNT_GYRO_ROLL) {
        ++count_gyro_roll;
    } else if (imu_data.gyro_roll < -THRESHOLD_COUNT_GYRO_ROLL) {
        --count_gyro_roll;
    } else {
        count_gyro_roll = 0;
    }
    if (imu_data.gyro_pitch > THRESHOLD_COUNT_GYRO_PITCH) {
        ++count_gyro_pitch;
    } else if (imu_data.gyro_pitch < -THRESHOLD_COUNT_GYRO_PITCH) {
        --count_gyro_pitch;
    } else {
        count_gyro_pitch = 0;
    }
    if (imu_data.gyro_yaw > THRESHOLD_COUNT_GYRO_YAW) {
        ++count_gyro_yaw;
    } else if (imu_data.gyro_yaw < -THRESHOLD_COUNT_GYRO_YAW) {
        --count_gyro_yaw;
    } else {
        count_gyro_yaw = 0;
    }
    if (count_gyro_roll == COUNT_GYRO_ROLL) {
        effect[count++] = UiInputEffect::GYRO_ROLL_PLUS;
    } else if (count_gyro_roll == -COUNT_GYRO_ROLL) {
        effect[count++] = UiInputEffect::GYRO_ROLL_MINUS;
    }
    if (count_gyro_pitch == COUNT_GYRO_PITCH) {
        effect[count++] = UiInputEffect::GYRO_PITCH_PLUS;
    } else if (count_gyro_pitch == -COUNT_GYRO_PITCH) {
        effect[count++] = UiInputEffect::GYRO_PITCH_MINUS;
    }

    // WALLSENSOR
    if (wallsensor_data.right > THRESHOLD_WALLSENSOR) {
        ++count_wallsensor_right;
    } else {
        count_wallsensor_right = 0;
    }
    if (wallsensor_data.left > THRESHOLD_WALLSENSOR) {
        ++count_wallsensor_left;
    } else {
        count_wallsensor_left = 0;
    }
    if (count_wallsensor_right == COUNT_WALLSENSOR) {
        effect[count++] = UiInputEffect::WALLSENSOR_RIGHT;
    }
    if (count_wallsensor_left == COUNT_WALLSENSOR) {
        effect[count++] = UiInputEffect::WALLSENSOR_LEFT;
    }

    // STABLE
    // clang-format off
    if (   misc::abs(imu_data.gyro_roll)  < THRESHOLD_STABLE_GYRO
        && misc::abs(imu_data.gyro_pitch) < THRESHOLD_STABLE_GYRO
        && misc::abs(imu_data.gyro_yaw)   < THRESHOLD_STABLE_GYRO) {
        ++count_stable;
    } else {
        count_stable = 0;
    }
    // clang-format on

    if (count_stable == 1000) {
        effect[count++] = UiInputEffect::STABLE_1SEC;
    } else if (count_stable == 3000) {
        effect[count++] = UiInputEffect::STABLE_3SEC;
    } else if (count_stable > 5000) {
        count_stable = 5000;
    }

    return count;
}

void Ui::interruptPeriodic() {
    // cmd::UI_OUT を受信
    // NOTE: 1ms ごとに 1回ずつ処理する
    static cmd::CommandFormatUiOut cmd_out;
    if (cmd_server->length(cmd::CommandId::UI_OUT) > 0) {
        cmd_server->pop(cmd::CommandId::UI_OUT, &cmd_out);
        output(cmd_out.type);
    }

    if (!enabled) return;

    // 入力を更新し、 cmd::UI_IN を送信
    static UiInputEffect ui_in[UI_IN_BUF_LENGTH] = {};
    auto count = input(ui_in);
    for (uint8_t i = 0; i < count; i++) {
        cmd::CommandFormatUiIn cmd_in = {
            .type = ui_in[i],
        };
        cmd_server->push(cmd::CommandId::UI_IN, &cmd_in);
    }
}

Ui* Ui::getInstance() {
    static Ui instance;
    return &instance;
}
