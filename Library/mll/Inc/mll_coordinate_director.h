//******************************************************************************
// @addtogroup MLL
// @file       mll_coordinate_director.h
// @brief      マウスの区画移動計画を立て、区画情報を持つクラス
//******************************************************************************
#pragma once

#include "mll_maze_solver.h"
#include "mll_operation_move_type.h"
#include "msg_server.h"

namespace mll {

class CoordinateDirector {
   private:
    CoordinateDirector();

    // mll::MouseSectionPosition current_section;

    msg::MessageServer* msg_server;

    // 目標座標を複数持つ
    MultiplePosition target_all_pos;
    // 現在座標 (区画のみ)
    MouseSectionPosition current_section;
    // 現在向かっている単一の座標
    misc::Point<uint16_t> target_pos;

    // 探索アルゴリズム
    AlgorithmType algorithm_type;
    MazeSolver* solver;

   public:
    // マウスが次に行うべき動作 (MoveType) を返す
    // この関数は、Localizer と OperationCoordinator からの情報をもとに判断する
    void getNextMove(OperationMoveCombination* moves, uint16_t& length);

    void setAlgorithm(AlgorithmType type);

    // 探索/最短走行が完了し、かつマウスが停止しているかどうかを返す
    bool isEnd();

    // マウスの現在の区画情報を返す？
    const MouseSectionPosition getCurrentSection() const;

    // マウスの目標区画情報を設定する
    void setTargetSection(MultiplePosition& pos);
    // マウスの現在の目標区画情報を返す
    // 到達済みの目標区画は削除されていくため、isEnd 時には空になるはず？
    const misc::Point<uint16_t> getTargetSection() const;

    // 引数 buf は 8450 Byte 以上のサイズを確保すること
    const uint16_t debugOutput(char* buf) const;

    static CoordinateDirector* getInstance();
};

}  // namespace mll
