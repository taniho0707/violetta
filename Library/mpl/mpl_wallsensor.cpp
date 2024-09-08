//******************************************************************************
// @addtogroup MPL
// @file       mpl_wallsensor.cpp
// @brief      壁センサー制御
//******************************************************************************
#include "mpl_wallsensor.h"

#include "hal_wallsensor.h"
#include "msg_format_wallsensor.h"
#include "msg_server.h"

mpl::WallSensor::WallSensor() {
    msg_server = msg::MessageServer::getInstance();
    params = misc::Params::getInstance()->getCachePointer();
}

void mpl::WallSensor::initPort() {
    hal::initWallSensorPort();
}

void mpl::WallSensor::deinitPort() {
    hal::deinitWallSensorPort();
}

mpl::MplStatus mpl::WallSensor::scanAllSync(hal::WallSensorData& data) {
    // 順番にすべてのチャンネルをスキャン
    hal::WallSensorData buffer_bundle = {0};
    hal::HalStatus result;

#ifdef MOUSE_LAZULI
    uint16_t buffer_bundle_array[WALLSENSOR_NUMS] = {0};
    result = hal::getWallSensorAllSync(buffer_bundle_array);
#endif  // MOUSE_LAZULI

#ifdef MOUSE_LAZULI_SENSOR
    uint16_t buffer_single = 0;
    uint16_t buffer_none = 0;
    uint16_t buffer_bundle_array[WALLSENSOR_NUMS] = {0};

    result = hal::getWallSensorSingleSync(buffer_bundle_array[static_cast<uint16_t>(hal::WallSensorNumbers::FRONTLEFT)],
                                          hal::WallSensorNumbers::FRONTLEFT);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::FRONTLEFT);
    for (int i = 0; i < 10000; ++i);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::FRONTLEFT);
    buffer_bundle.FRONTLEFT = buffer_single - buffer_none;
    if (buffer_bundle.FRONTLEFT > 4096) buffer_bundle.FRONTLEFT = 0;

    result = hal::getWallSensorSingleSync(buffer_bundle_array[static_cast<uint16_t>(hal::WallSensorNumbers::LEFT)], hal::WallSensorNumbers::LEFT);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::LEFT);
    for (int i = 0; i < 10000; ++i);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::LEFT);
    buffer_bundle.LEFT = buffer_single - buffer_none;
    if (buffer_bundle.LEFT > 4096) buffer_bundle.LEFT = 0;

    result = hal::getWallSensorSingleSync(buffer_bundle_array[static_cast<uint16_t>(hal::WallSensorNumbers::CENTER)], hal::WallSensorNumbers::CENTER);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::CENTER);
    for (int i = 0; i < 10000; ++i);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::CENTER);
    buffer_bundle.CENTER = buffer_single - buffer_none;
    if (buffer_bundle.CENTER > 4096) buffer_bundle.CENTER = 0;

    result = hal::getWallSensorSingleSync(buffer_bundle_array[static_cast<uint16_t>(hal::WallSensorNumbers::RIGHT)], hal::WallSensorNumbers::RIGHT);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::RIGHT);
    for (int i = 0; i < 10000; ++i);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::RIGHT);
    buffer_bundle.RIGHT = buffer_single - buffer_none;
    if (buffer_bundle.RIGHT > 4096) buffer_bundle.RIGHT = 0;

    result = hal::getWallSensorSingleSync(buffer_bundle_array[static_cast<uint16_t>(hal::WallSensorNumbers::FRONTRIGHT)],
                                          hal::WallSensorNumbers::FRONTRIGHT);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::FRONTRIGHT);
    for (int i = 0; i < 10000; ++i);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::FRONTRIGHT);
    buffer_bundle.FRONTRIGHT = buffer_single - buffer_none;
    if (buffer_bundle.FRONTRIGHT > 4096) buffer_bundle.FRONTRIGHT = 0;
#endif  // MOUSE_LAZULI_SENSOR

#ifdef MOUSE_ZIRCONIA2KAI
    uint16_t buffer_single = 0;
    uint16_t buffer_none = 0;

    // FRONTLEFT
    result = hal::getWallSensorSingleSync(buffer_none, hal::WallSensorNumbers::FRONTLEFT);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::FRONTLEFT);
    mpl::Timer::sleepNs(params->wallsensor_turnon);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::FRONTLEFT);
    buffer_bundle.FRONTLEFT = buffer_single - buffer_none;
    if (buffer_bundle.FRONTLEFT > 4096) buffer_bundle.FRONTLEFT = 0;

    // LEFT
    result = hal::getWallSensorSingleSync(buffer_none, hal::WallSensorNumbers::LEFT);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::LEFT);
    mpl::Timer::sleepNs(params->wallsensor_turnon);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::LEFT);
    buffer_bundle.LEFT = buffer_single - buffer_none;
    if (buffer_bundle.LEFT > 4096) buffer_bundle.LEFT = 0;

    // RIGHT
    result = hal::getWallSensorSingleSync(buffer_none, hal::WallSensorNumbers::RIGHT);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::RIGHT);
    mpl::Timer::sleepNs(params->wallsensor_turnon);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::RIGHT);
    buffer_bundle.RIGHT = buffer_single - buffer_none;
    if (buffer_bundle.RIGHT > 4096) buffer_bundle.RIGHT = 0;

    // FRONTRIGHT
    result = hal::getWallSensorSingleSync(buffer_none, hal::WallSensorNumbers::FRONTRIGHT);
    hal::setWallSensorLedOn(hal::WallSensorNumbers::FRONTRIGHT);
    mpl::Timer::sleepNs(params->wallsensor_turnon);
    result = hal::getWallSensorSingleSync(buffer_single, hal::WallSensorNumbers::FRONTRIGHT);
    buffer_bundle.FRONTRIGHT = buffer_single - buffer_none;
    if (buffer_bundle.FRONTRIGHT > 4096) buffer_bundle.FRONTRIGHT = 0;

    hal::setWallSensorLedOff();
#endif  // MOUSE_ZIRCONIA2KAI

    // 結果を格納
    if (result == hal::HalStatus::SUCCESS) {
#ifdef MOUSE_VIOLETTA
        data.FRONTLEFT = buffer[0];
        data.LEFT = buffer[1];
        data.FRONT = buffer[2];
        data.RIGHT = buffer[3];
        data.FRONTRIGHT = buffer[4];
#endif
#ifdef MOUSE_LAZULI
        buffer_bundle.FRONTLEFT = buffer_bundle_array[0];
        buffer_bundle.LEFT = buffer_bundle_array[1];
        buffer_bundle.CENTER = buffer_bundle_array[2];
        buffer_bundle.RIGHT = buffer_bundle_array[3];
        buffer_bundle.FRONTRIGHT = buffer_bundle_array[4];
#endif
#ifdef MOUSE_LAZULI_SENSOR
        buffer_bundle.FRONTLEFT = buffer_bundle_array[0];
        buffer_bundle.LEFT = buffer_bundle_array[1];
        buffer_bundle.CENTER = buffer_bundle_array[2];
        buffer_bundle.RIGHT = buffer_bundle_array[3];
        buffer_bundle.FRONTRIGHT = buffer_bundle_array[4];
#endif
#ifdef MOUSE_ZIRCONIA2KAI
        data.FRONTLEFT = buffer_bundle.FRONTLEFT;
        data.LEFT = buffer_bundle.LEFT;
        data.RIGHT = buffer_bundle.RIGHT;
        data.FRONTRIGHT = buffer_bundle.FRONTRIGHT;
#endif
        return mpl::MplStatus::SUCCESS;
    } else {
        return mpl::MplStatus::ERROR;
    }
}

void mpl::WallSensor::interruptPeriodic() {
    scanAllSync(last);

#ifdef MOUSE_LAZULI
    msg_format.frontleft = last.FRONTLEFT;
    msg_format.left = last.LEFT;
    msg_format.center = last.CENTER;
    msg_format.right = last.RIGHT;
    msg_format.frontright = last.FRONTRIGHT;
#elif MOUSE_LAZULI_SENSOR
    msg_format.frontleft = last.FRONTLEFT;
    msg_format.left = last.LEFT;
    msg_format.center = last.CENTER;
    msg_format.right = last.RIGHT;
    msg_format.frontright = last.FRONTRIGHT;
#elif defined(MOUSE_ZIRCONIA2KAI)
    msg_format.frontleft = last.FRONTLEFT;
    msg_format.left = last.LEFT;
    msg_format.right = last.RIGHT;
    msg_format.frontright = last.FRONTRIGHT;
#endif
    msg_server->sendMessage(msg::ModuleId::WALLSENSOR, &msg_format);
}

mpl::WallSensor* mpl::WallSensor::getInstance() {
    static mpl::WallSensor instance;
    return &instance;
}
