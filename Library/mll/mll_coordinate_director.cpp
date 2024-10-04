//******************************************************************************
// @addtogroup MLL
// @file       mll_coordinate_director.cpp
// @brief      マウスの区画移動計画を立て、区画情報を持つクラス
//******************************************************************************
#include "mll_coordinate_director.h"

using namespace mll;

CoordinateDirector::CoordinateDirector() {
    maze_solver = MazeSolver::getInstance();
    current_section = MouseSectionPosition{0, 1, CardinalDirection::NORTH};
}

// マウスが次に行うべき動作 (MoveType) を返す
// この関数は、Localizer と OperationCoordinator からの情報をもとに判断する
OperationMoveCombination CoordinateDirector::getNextMove() {}

bool CoordinateDirector::isEnd() {}

// マウスの現在の目標区画情報を返す
const MouseSectionPosition CoordinateDirector::getTargetSection() const {}

CoordinateDirector* CoordinateDirector::getInstance() {
    static CoordinateDirector instance;
    return &instance;
}
