//******************************************************************************
// @addtogroup MPL
// @file       mpl_wallsensor.cpp
// @brief      壁センサー制御
//******************************************************************************
#include "mpl_wallsensor.h"

#include "mpl_timer.h"
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
    uint16_t buffer_single = 0;
    uint16_t buffer_none = 0;
    hal::HalStatus result;

#ifdef MOUSE_LAZULI
    // FRONTLEFT
    hal::setWallSensorChargeStart();
    // mpl::Timer::sleepNs(50);
    // hal::setWallSensorChargeStop();

    hal::setWallSensorAdcSelect(hal::WallSensorNumbers::FRONTLEFT);
    mpl::Timer::sleepNs(50);
    hal::startWallSensorConversion();
    mpl::Timer::sleepNs(600);
    hal::getWallSensorSingleSync(buffer_single);  // 破棄

    hal::setWallSensorLedOn(hal::WallSensorNumbers::FRONTLEFT);
    mpl::Timer::sleepNs(50);
    hal::startWallSensorConversion();
    // hal::setWallSensorLedOff(hal::WallSensorNumbers::FRONTLEFT);
    mpl::Timer::sleepNs(600);
    result = hal::getWallSensorSingleSync(buffer_single);
    mpl::Timer::sleepNs(50);
    buffer_bundle.FRONTLEFT = buffer_single;

    // LEFT
    // hal::setWallSensorChargeStart();
    // mpl::Timer::sleepNs(50);
    // hal::setWallSensorChargeStop();

    hal::setWallSensorAdcSelect(hal::WallSensorNumbers::LEFT);
    mpl::Timer::sleepNs(50);
    hal::startWallSensorConversion();
    mpl::Timer::sleepNs(600);
    hal::getWallSensorSingleSync(buffer_single);  // 破棄

    hal::setWallSensorLedOn(hal::WallSensorNumbers::LEFT);
    mpl::Timer::sleepNs(50);
    hal::startWallSensorConversion();
    hal::setWallSensorLedOff(hal::WallSensorNumbers::LEFT);
    mpl::Timer::sleepNs(600);
    result = hal::getWallSensorSingleSync(buffer_single);
    mpl::Timer::sleepNs(50);
    buffer_bundle.LEFT = buffer_single;

    // RIGHT
    // hal::setWallSensorChargeStart();
    // mpl::Timer::sleepNs(50);
    // hal::setWallSensorChargeStop();

    hal::setWallSensorAdcSelect(hal::WallSensorNumbers::RIGHT);
    mpl::Timer::sleepNs(50);
    hal::startWallSensorConversion();
    mpl::Timer::sleepNs(600);
    hal::getWallSensorSingleSync(buffer_single);  // 破棄

    hal::setWallSensorLedOn(hal::WallSensorNumbers::RIGHT);
    mpl::Timer::sleepNs(50);
    hal::startWallSensorConversion();
    hal::setWallSensorLedOff(hal::WallSensorNumbers::RIGHT);
    mpl::Timer::sleepNs(600);
    result = hal::getWallSensorSingleSync(buffer_single);
    mpl::Timer::sleepNs(50);
    buffer_bundle.RIGHT = buffer_single;

    // FRONTRIGHT
    // hal::setWallSensorChargeStart();
    // mpl::Timer::sleepNs(50);
    // hal::setWallSensorChargeStop();

    hal::setWallSensorAdcSelect(hal::WallSensorNumbers::FRONTRIGHT);
    mpl::Timer::sleepNs(50);
    hal::startWallSensorConversion();
    mpl::Timer::sleepNs(600);
    hal::getWallSensorSingleSync(buffer_single);  // 破棄

    hal::setWallSensorLedOn(hal::WallSensorNumbers::FRONTRIGHT);
    mpl::Timer::sleepNs(50);
    hal::startWallSensorConversion();
    hal::setWallSensorLedOff(hal::WallSensorNumbers::FRONTRIGHT);
    mpl::Timer::sleepNs(600);
    result = hal::getWallSensorSingleSync(buffer_single);
    mpl::Timer::sleepNs(50);
    buffer_bundle.FRONTRIGHT = buffer_single;
#endif  // MOUSE_LAZULI

#ifdef MOUSE_ZIRCONIA2KAI
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
        data.FRONTLEFT = buffer_bundle.FRONTLEFT;
        data.LEFT = buffer_bundle.LEFT;
        data.RIGHT = buffer_bundle.RIGHT;
        data.FRONTRIGHT = buffer_bundle.FRONTRIGHT;
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

    msg_format.frontleft = last.FRONTLEFT;
    msg_format.left = last.LEFT;
    msg_format.right = last.RIGHT;
    msg_format.frontright = last.FRONTRIGHT;
    msg_server->sendMessage(msg::ModuleId::WALLSENSOR, &msg_format);
}

mpl::WallSensor* mpl::WallSensor::getInstance() {
    static mpl::WallSensor instance;
    return &instance;
}
