//******************************************************************************
// @addtogroup MLL
// @file       mll_wall.cpp
// @brief      壁データの定義
//******************************************************************************
#include "mll_wall.h"

#include "util.h"

using namespace mll;

void Walldata::operator=(Walldata input) {
    data = input.data;
}

void Walldata::operator+=(Walldata input) {
    data |= input.data;
}

void Walldata::operator|=(Walldata input) {
    data |= input.data;
}

bool Walldata::isExistWall(FirstPersonDirection dir) {
    uint8_t mask = static_cast<uint8_t>(dir);
    return (data & mask) != 0;
}

void Walldata::addWall(FirstPersonDirection dir) {
    data |= static_cast<uint8_t>(dir);
}

void Walldata::removeWall(FirstPersonDirection dir) {
    data &= ~static_cast<uint8_t>(dir);
}
