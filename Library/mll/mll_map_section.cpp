//******************************************************************************
// @addtogroup MLL
// @file       mll_map_section.cpp
// @brief      迷路の区画マップ
//******************************************************************************
#include "mll_map_section.h"

using namespace mll;

void Map::format() {
    formatWall();
    formatReached();
    goals.clear();
}

void Map::formatWall() {
    for (auto &i : column) {
        i = 0;
    }
    for (auto &i : row) {
        i = 0;
    }
}

void Map::formatReached() {
    for (auto &i : reached) {
        i = 0;
    }
}

void Map::addGoal(int16_t x, int16_t y) {
    goals.add(x, y);
}

void Map::addWall(int8_t x, int8_t y, CardinalDirection angle, Walldata wall) {
    Walldata tmp = Walldata::rotateWallToAbsolute(wall, angle);
    if (tmp.isExistWall(FirstPersonDirection::FRONT)) addSingleWall(x, y, CardinalDirection::NORTH);
    if (tmp.isExistWall(FirstPersonDirection::LEFT)) addSingleWall(x, y, CardinalDirection::WEST);
    if (tmp.isExistWall(FirstPersonDirection::RIGHT)) addSingleWall(x, y, CardinalDirection::EAST);
    if (tmp.isExistWall(FirstPersonDirection::BACK)) addSingleWall(x, y, CardinalDirection::SOUTH);
}

void Map::setWall(int8_t x, int8_t y, CardinalDirection angle, Walldata wall) {
    Walldata tmp = Walldata::rotateWallToAbsolute(wall, angle);
    setSingleWall(x, y, CardinalDirection::NORTH, tmp.isExistWall(FirstPersonDirection::FRONT));
    setSingleWall(x, y, CardinalDirection::EAST, tmp.isExistWall(FirstPersonDirection::RIGHT));
    setSingleWall(x, y, CardinalDirection::SOUTH, tmp.isExistWall(FirstPersonDirection::BACK));
    setSingleWall(x, y, CardinalDirection::WEST, tmp.isExistWall(FirstPersonDirection::LEFT));
}

void Map::addSingleWall(int8_t x, int8_t y, CardinalDirection angle) {
    if ((x == 0 && angle == CardinalDirection::WEST) || (x == 31 && angle == CardinalDirection::EAST) ||
        (y == 0 && angle == CardinalDirection::SOUTH) || (y == 31 && angle == CardinalDirection::NORTH))
        return;
    if (angle == CardinalDirection::NORTH) {
        row[y] |= (0x80000000 >> x);
    } else if (angle == CardinalDirection::EAST) {
        column[x] |= (0x80000000 >> y);
    } else if (angle == CardinalDirection::SOUTH) {
        row[y - 1] |= (0x80000000 >> x);
    } else if (angle == CardinalDirection::WEST) {
        column[x - 1] |= (0x80000000 >> y);
    }
}

void Map::setSingleWall(int8_t x, int8_t y, CardinalDirection angle, bool wall) {
    if ((x == 0 && angle == CardinalDirection::WEST) || (x == 31 && angle == CardinalDirection::EAST) ||
        (y == 0 && angle == CardinalDirection::SOUTH) || (y == 31 && angle == CardinalDirection::NORTH))
        return;
    if (wall) {
        addSingleWall(x, y, angle);
    } else {
        if (angle == CardinalDirection::NORTH) {
            row[y] &= ~(0x80000000 >> x);
        } else if (angle == CardinalDirection::EAST) {
            column[x] &= ~(0x80000000 >> y);
        } else if (angle == CardinalDirection::SOUTH) {
            row[y - 1] &= ~(0x80000000 >> x);
        } else if (angle == CardinalDirection::WEST) {
            column[x - 1] &= ~(0x80000000 >> y);
        }
    }
}

Walldata Map::getWalldata(int8_t x, int8_t y) {
    Walldata wall;
    if (isExistWall(x, y, CardinalDirection::NORTH)) wall.addWall(FirstPersonDirection::FRONT);
    if (isExistWall(x, y, CardinalDirection::SOUTH)) wall.addWall(FirstPersonDirection::BACK);
    if (isExistWall(x, y, CardinalDirection::EAST)) wall.addWall(FirstPersonDirection::RIGHT);
    if (isExistWall(x, y, CardinalDirection::WEST)) wall.addWall(FirstPersonDirection::LEFT);
    return wall;
}

Walldata Map::getKnownWalldata(int8_t x, int8_t y) {
    Walldata wall;
    if (isExistWall(x, y, CardinalDirection::NORTH) || hasWatched(x, y, CardinalDirection::NORTH) == false) wall.addWall(FirstPersonDirection::FRONT);
    if (isExistWall(x, y, CardinalDirection::SOUTH) || hasWatched(x, y, CardinalDirection::SOUTH) == false) wall.addWall(FirstPersonDirection::BACK);
    if (isExistWall(x, y, CardinalDirection::EAST) || hasWatched(x, y, CardinalDirection::EAST) == false) wall.addWall(FirstPersonDirection::RIGHT);
    if (isExistWall(x, y, CardinalDirection::WEST) || hasWatched(x, y, CardinalDirection::WEST) == false) wall.addWall(FirstPersonDirection::LEFT);
    return wall;
}

bool Map::isExistWall(int8_t x, int8_t y, CardinalDirection angle) {
    uint32_t ans = 0;
    if ((x == 0 && angle == CardinalDirection::WEST) || (x == 31 && angle == CardinalDirection::EAST) ||
        (y == 0 && angle == CardinalDirection::SOUTH) || (y == 31 && angle == CardinalDirection::NORTH))
        return true;
    if (x < 0 || x > 31 || y < 0 || y > 31) return true;
    if (angle == CardinalDirection::NORTH) {
        ans = row[y] & (0x80000000 >> x);
    } else if (angle == CardinalDirection::EAST) {
        ans = column[x] & (0x80000000 >> y);
    } else if (angle == CardinalDirection::SOUTH) {
        ans = row[y - 1] & (0x80000000 >> x);
    } else if (angle == CardinalDirection::WEST) {
        ans = column[x - 1] & (0x80000000 >> y);
    } else {
        ans = 1;
    }
    if (ans > 0) return true;
    else return false;
}

// bool Map::isExistWallFromMouse(int8_t x, int8_t y, CardinalDirection angle, int8_t mousex, int8_t mousey, FirstPersonDirection mouseangle) {
//     if (mouseangle == CardinalDirection::NORTH) {
//         return isExistWall(x + mousex, y + mousey, angle);
//     } else if (mouseangle == CardinalDirection::EAST) {
//         return isExistWall(x + mousey, y - mousex, static_cast<CardinalDirection>((static_cast<uint8_t>(angle) + 1) % 4));
//     } else if (mouseangle == CardinalDirection::SOUTH) {
//         return isExistWall(x - mousex, y - mousey, static_cast<CardinalDirection>((static_cast<uint8_t>(angle) + 2) % 4));
//     } else {  // WEST
//         return isExistWall(x - mousey, y + mousex, static_cast<CardinalDirection>((static_cast<uint8_t>(angle) + 3) % 4));
//     }
// }

void Map::setReached(int8_t x, int8_t y) {
    if (x > 31 || x < 0 || y > 31 || y < 0) return;
    reached[y] |= (0x80000000 >> x);
    return;
}

bool Map::hasReached(int8_t x, int8_t y) {
    if (x < 0 || x > 31 || y < 0 || y > 31) return false;
    if (reached[y] & (0x80000000 >> x)) return true;
    // else if(
    // 	(reached[y] & (0x80000000 >> (x+1)))
    // 	|| (x==0 ? false : (reached[y] & (0x80000000 >> (x-1))))
    // 	|| (y==31 ? false : (reached[y+1] & (0x80000000 >> x)))
    // 	|| (y==0 ? false : (reached[y-1] & (0x80000000 >> x)))
    // 	) return true;
    else return false;
}

bool Map::hasWatched(int8_t x, int8_t y, CardinalDirection angle) {
    switch (angle) {
        case CardinalDirection::NORTH:
            return (hasReached(x, y) || hasReached(x, y + 1));
        case CardinalDirection::EAST:
            return (hasReached(x, y) || hasReached(x + 1, y));
        case CardinalDirection::SOUTH:
            return (hasReached(x, y) || hasReached(x, y - 1));
        case CardinalDirection::WEST:
            return (hasReached(x, y) || hasReached(x - 1, y));
        default:
            return false;
    }
}

void Map::copyFrom(const Map &m) {
    for (int i = 0; i < 31; i++) {
        column[i] = m.column[i];
        row[i] = m.row[i];
    }
    for (int i = 0; i < 32; i++) {
        reached[i] = m.reached[i];
    }
    goals = m.goals;
}
