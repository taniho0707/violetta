//******************************************************************************
// @addtogroup MLL
// @file       mll_coordinate_director.h
// @brief      マウスの区画移動計画を立て、区画情報を持つクラス
//******************************************************************************
#pragma once

#include "mll_maze_solver.h"
#include "mll_operation_move_type.h"

namespace mll {

class CoordinateDirector {
   private:
    CoordinateDirector();

    mll::MazeSolver* maze_solver;

    mll::MouseSectionPosition current_section;

   public:
    // マウスが次に行うべき動作 (MoveType) を返す
    // この関数は、Localizer と OperationCoordinator からの情報をもとに判断する
    OperationMoveCombination getNextMove();

    // マウスの現在の区画情報を返す？

    // マウスの現在の目標区画情報を返す
    const MouseSectionPosition getTargetSection() const;

    static CoordinateDirector* getInstance();
};

}  // namespace mll
