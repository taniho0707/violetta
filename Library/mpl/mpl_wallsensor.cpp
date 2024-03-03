//******************************************************************************
// @addtogroup MPL
// @file       mpl_wallsensor.cpp
// @brief      壁センサー制御
//******************************************************************************
#include "mpl_wallsensor.h"

mpl::WallSensor::WallSensor() { hal::initWallSensorPort(); }

void mpl::WallSensor::initPort() { hal::initWallSensorPort(); }

void mpl::WallSensor::deinitPort() { hal::deinitWallSensorPort(); }

mpl::MplStatus mpl::WallSensor::scanAllSync(hal::WallSensorData& data) {
    hal::WallSensorData buffer = {0};
    hal::HalStatus result = hal::getWallSensorAllSync(buffer);
    if (result == hal::HalStatus::SUCCESS) {
#ifdef MOUSE_VIOLETTA
        data.FRONTLEFT = buffer[0];
        data.LEFT = buffer[1];
        data.FRONT = buffer[2];
        data.RIGHT = buffer[3];
        data.FRONTRIGHT = buffer[4];
#endif
#ifdef MOUSE_ZIRCONIA2KAI
        data.FRONTLEFT = buffer.FRONTLEFT;
        data.LEFT = buffer.LEFT;
        data.RIGHT = buffer.RIGHT;
        data.FRONTRIGHT = buffer.FRONTRIGHT;
#endif
        return mpl::MplStatus::SUCCESS;
    } else {
        return mpl::MplStatus::ERROR;
    }
}

void mpl::WallSensor::interrupt() {}

mpl::WallSensor* mpl::WallSensor::getInstance() {
    static mpl::WallSensor instance;
    return &instance;
}
