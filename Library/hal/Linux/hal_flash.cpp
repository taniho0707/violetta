//******************************************************************************
// @addtogroup HAL
// @file       hal_flash.cpp
// @brief      内蔵FLASH制御
//******************************************************************************
#include "hal_flash.h"

hal::HalStatus hal::initInternalFlash() {
    return HalStatus::SUCCESS;
}

hal::HalStatus hal::deinitInternalFlash() {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus writeInternalFlash(uint32_t address, uint32_t data) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus writeInternalFlash(uint32_t address, uint32_t* data, uint32_t size) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus readInternalFlash(uint32_t address, uint32_t& data) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus readInternalFlash(uint32_t address, uint32_t* data, uint32_t size) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus eraseInternalFlashSync(hal::InternalFlashSector sector) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus eraseInternalFlashAsync(hal::InternalFlashSector sector) {
    return hal::HalStatus::SUCCESS;
}

hal::HalStatus isDoneEraseInternalFlash() {
    return hal::HalStatus::SUCCESS;
}
