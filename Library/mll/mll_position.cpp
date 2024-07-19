//******************************************************************************
// @addtogroup MLL
// @file       mll_position.cpp
// @brief      位置や向きに関する定義
//******************************************************************************
#include "mll_position.h"

#include "util.h"

using namespace mll;

MultiplePosition::MultiplePosition() {
    stored = 0;
}

void MultiplePosition::clear() {
    stored = 0;
}

bool MultiplePosition::add(int16_t x, int16_t y) {
    uint32_t mask = 1;
    for (uint8_t i = 0; i < MAX_LENGTH_MULTIPLE_POSITION; i++) {
        if ((stored & mask) == 0) {
            curs[i] = misc::Point<uint16_t>{x, y};
            stored |= mask;
            return true;
        }
        mask <<= 1;
    }
    return false;
}

void MultiplePosition::remove(int16_t x, int16_t y) {
    uint32_t mask = 1;
    for (uint8_t i = 0; i < MAX_LENGTH_MULTIPLE_POSITION; i++) {
        if (curs[i].x == x && curs[i].y == y) {
            stored &= ~mask;
            return;
        }
        mask <<= 1;
    }
}

uint8_t MultiplePosition::length() {
    uint8_t count = 0;
    uint32_t mask = 1;
    for (uint8_t i = 0; i < MAX_LENGTH_MULTIPLE_POSITION; i++) {
        if ((stored & mask) != 0) {
            count++;
        }
        mask <<= 1;
    }
    return count;
}

bool MultiplePosition::isInclude(misc::Point<uint16_t> cur) {
    uint32_t mask = 1;
    for (uint8_t i = 0; i < MAX_LENGTH_MULTIPLE_POSITION; i++) {
        if ((stored & mask) != 0) {
            if (curs[i].x == cur.x && curs[i].y == cur.y) {
                return true;
            }
        }
        mask <<= 1;
    }
    return false;
}

bool MultiplePosition::hasData(uint8_t index) {
    return (stored & (1 << index)) != 0;
}

misc::Point<uint16_t>* MultiplePosition::begin() {
    for (uint8_t i = 0; i < MAX_LENGTH_MULTIPLE_POSITION; i++) {
        if (hasData(i)) {
            return &curs[i];
        }
    }
    return nullptr;
}

misc::Point<uint16_t>* MultiplePosition::end() {
    for (uint8_t i = MAX_LENGTH_MULTIPLE_POSITION - 1; i >= 0; i--) {
        if (hasData(i)) {
            return &curs[i + 1];
        }
    }
    return nullptr;
}
