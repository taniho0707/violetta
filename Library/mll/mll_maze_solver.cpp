//******************************************************************************
// @addtogroup MLL
// @file       mll_maze_solver.cpp
// @brief      迷路解析用のクラス
//******************************************************************************
#include "mll_maze_solver.h"

#include <queue>

#include "mll_footmap.h"
#include "mll_map_section.h"
#include "util.h"

using namespace mll;

MazeSolver::MazeSolver() {
    footmap1.clear();
    footmap2.clear();
    footmap_index = 1;
    map.format();
    destination.clear();
}

void MazeSolver::setAlgorithm(AlgorithmType algorithm) {
    this->algorithm = algorithm;
}

void MazeSolver::updateFootmapAdachi(Footmap* fm, int8_t current_x, int8_t current_y) {
    misc::Point<int8_t> buf;
    misc::Point<int8_t> tmp;

    bool is_end = false;

    fm->clear();
    misc::Queue<misc::Point<int8_t>, 32> que;

    for (auto ite : map.goals) {
        tmp.x = ite.x;
        tmp.y = ite.y;
        que.push(tmp);
        fm->set(ite.x, ite.y, 0);
    }

    while (!que.empty()) {
        buf = que.front();
        que.pop();

        if ((map.isExistWall(buf.x, buf.y, CardinalDirection::NORTH) == false) && (fm->get(buf.x, buf.y + 1, 0) == 1024)) {
            tmp.x = buf.y;
            tmp.x = buf.y + 1;
            if (!is_end) que.push(tmp);
            fm->set(buf.x, buf.y + 1, fm->getMinNextTo(buf.x, buf.y + 1, map.getWalldata(buf.x, buf.y + 1)) + 1);
        }
        if ((map.isExistWall(buf.x, buf.y, CardinalDirection::EAST) == false) && (fm->get(buf.x + 1, buf.y, 0) == 1024)) {
            tmp.x = buf.x + 1;
            tmp.y = buf.y;
            if (!is_end) que.push(tmp);
            fm->set(buf.x + 1, buf.y, fm->getMinNextTo(buf.x + 1, buf.y, map.getWalldata(buf.x + 1, buf.y)) + 1);
        }
        if ((map.isExistWall(buf.x, buf.y, CardinalDirection::SOUTH) == false) && (fm->get(buf.x, buf.y - 1, 0) == 1024)) {
            tmp.x = buf.x;
            tmp.y = buf.y - 1;
            if (!is_end) que.push(tmp);
            fm->set(buf.x, buf.y - 1, fm->getMinNextTo(buf.x, buf.y - 1, map.getWalldata(buf.x, buf.y - 1)) + 1);
        }
        if ((map.isExistWall(buf.x, buf.y, CardinalDirection::WEST) == false) && (fm->get(buf.x - 1, buf.y, 0) == 1024)) {
            tmp.x = buf.x - 1;
            tmp.y = buf.y;
            if (!is_end) que.push(tmp);
            fm->set(buf.x - 1, buf.y, fm->getMinNextTo(buf.x - 1, buf.y, map.getWalldata(buf.x - 1, buf.y)) + 1);
        }

        if (buf.x == current_x && buf.y == current_y) {
            is_end = true;
            return;
        }
    }
}

uint8_t MazeSolver::updateFootmap(int8_t current_x, int8_t current_y) {
    Footmap* fm;
    if (footmap_index == 1) {
        fm = &footmap2;
    } else {
        fm = &footmap1;
    }

    switch (algorithm) {
        case AlgorithmType::LEFT_HAND:
            break;
        case AlgorithmType::ADACHI:
            updateFootmapAdachi(fm, current_x, current_y);
            break;
        default:
            break;
    }

    footmap_index = (footmap_index == 1) ? 2 : 1;
    return footmap_index;
}

void MazeSolver::clearFootmap() {
    // TODO: updateFootmap中に呼び出しても問題ないようにしておく
    footmap1.clear();
    footmap2.clear();
    footmap_index = 1;
}

Footmap* MazeSolver::getFootmap() {
    return (footmap_index == 1) ? &footmap1 : &footmap2;
}

FirstPersonDirection MazeSolver::getNextDirectionInSearchLeftHand(int8_t current_x, int8_t current_y, CardinalDirection current_angle) {
    if (current_angle == CardinalDirection::NORTH) {
        if (!map.isExistWall(current_x, current_y, CardinalDirection::WEST)) {
            return FirstPersonDirection::LEFT;
        } else if (!map.isExistWall(current_x, current_y, CardinalDirection::NORTH)) {
            return FirstPersonDirection::FRONT;
        } else if (!map.isExistWall(current_x, current_y, CardinalDirection::EAST)) {
            return FirstPersonDirection::RIGHT;
        } else {
            return FirstPersonDirection::BACK;
        }
    } else if (current_angle == CardinalDirection::WEST) {
        if (!map.isExistWall(current_x, current_y, CardinalDirection::SOUTH)) {
            return FirstPersonDirection::LEFT;
        } else if (!map.isExistWall(current_x, current_y, CardinalDirection::WEST)) {
            return FirstPersonDirection::FRONT;
        } else if (!map.isExistWall(current_x, current_y, CardinalDirection::NORTH)) {
            return FirstPersonDirection::RIGHT;
        } else {
            return FirstPersonDirection::BACK;
        }
    } else if (current_angle == CardinalDirection::SOUTH) {
        if (!map.isExistWall(current_x, current_y, CardinalDirection::EAST)) {
            return FirstPersonDirection::LEFT;
        } else if (!map.isExistWall(current_x, current_y, CardinalDirection::SOUTH)) {
            return FirstPersonDirection::FRONT;
        } else if (!map.isExistWall(current_x, current_y, CardinalDirection::WEST)) {
            return FirstPersonDirection::RIGHT;
        } else {
            return FirstPersonDirection::BACK;
        }
    } else if (current_angle == CardinalDirection::EAST) {
        if (!map.isExistWall(current_x, current_y, CardinalDirection::NORTH)) {
            return FirstPersonDirection::LEFT;
        } else if (!map.isExistWall(current_x, current_y, CardinalDirection::EAST)) {
            return FirstPersonDirection::FRONT;
        } else if (!map.isExistWall(current_x, current_y, CardinalDirection::SOUTH)) {
            return FirstPersonDirection::RIGHT;
        } else {
            return FirstPersonDirection::BACK;
        }
    } else {
        // 想定外
        return FirstPersonDirection::BACK;
    }
}

FirstPersonDirection MazeSolver::getNextDirectionInSearchAdachi(int8_t current_x, int8_t current_y, CardinalDirection current_angle) {
    Footmap* fm = getFootmap();
    Walldata walldata = map.getKnownWalldata(current_x, current_y);
    uint16_t min = fm->getMinNextTo(current_x, current_y, walldata);
    if (current_angle == CardinalDirection::NORTH) {
        if (fm->get(current_x, current_y + 1, 1024) == min && walldata.isExistWall(FirstPersonDirection::FRONT) == false)
            return FirstPersonDirection::FRONT;
        else if (fm->get(current_x + 1, current_y, 1024) == min && walldata.isExistWall(FirstPersonDirection::RIGHT) == false)
            return FirstPersonDirection::RIGHT;
        else if (fm->get(current_x - 1, current_y, 1024) == min && walldata.isExistWall(FirstPersonDirection::LEFT) == false)
            return FirstPersonDirection::LEFT;
        else return FirstPersonDirection::BACK;
    } else if (current_angle == CardinalDirection::EAST) {
        if (fm->get(current_x + 1, current_y, 1024) == min && walldata.isExistWall(FirstPersonDirection::RIGHT) == false)
            return FirstPersonDirection::FRONT;
        else if (fm->get(current_x, current_y - 1, 1024) == min && walldata.isExistWall(FirstPersonDirection::BACK) == false)
            return FirstPersonDirection::RIGHT;
        else if (fm->get(current_x, current_y + 1, 1024) == min && walldata.isExistWall(FirstPersonDirection::FRONT) == false)
            return FirstPersonDirection::LEFT;
        else return FirstPersonDirection::BACK;
    } else if (current_angle == CardinalDirection::SOUTH) {
        if (fm->get(current_x, current_y - 1, 1024) == min && walldata.isExistWall(FirstPersonDirection::BACK) == false)
            return FirstPersonDirection::FRONT;
        else if (fm->get(current_x - 1, current_y, 1024) == min && walldata.isExistWall(FirstPersonDirection::LEFT) == false)
            return FirstPersonDirection::RIGHT;
        else if (fm->get(current_x + 1, current_y, 1024) == min && walldata.isExistWall(FirstPersonDirection::RIGHT) == false)
            return FirstPersonDirection::LEFT;
        else return FirstPersonDirection::BACK;
    } else if (current_angle == CardinalDirection::WEST) {
        if (fm->get(current_x - 1, current_y, 1024) == min && walldata.isExistWall(FirstPersonDirection::LEFT) == false)
            return FirstPersonDirection::FRONT;
        else if (fm->get(current_x, current_y + 1, 1024) == min && walldata.isExistWall(FirstPersonDirection::FRONT) == false)
            return FirstPersonDirection::RIGHT;
        else if (fm->get(current_x, current_y - 1, 1024) == min && walldata.isExistWall(FirstPersonDirection::BACK) == false)
            return FirstPersonDirection::LEFT;
        else return FirstPersonDirection::BACK;
    }
    return FirstPersonDirection::BACK;
}

FirstPersonDirection MazeSolver::getNextDirectionInSearch(int8_t current_x, int8_t current_y, CardinalDirection current_angle) {
    switch (algorithm) {
        case AlgorithmType::LEFT_HAND:
            return getNextDirectionInSearchLeftHand(current_x, current_y, current_angle);
        case AlgorithmType::ADACHI:
            return getNextDirectionInSearchAdachi(current_x, current_y, current_angle);
        default:
            return FirstPersonDirection::FRONT;
    }
}

MazeSolver* MazeSolver::getInstance() {
    static MazeSolver instance;
    return &instance;
}
