//******************************************************************************
// @addtogroup MLL
// @file       mll_maze_solver.h
// @brief      迷路解析用のクラス
//******************************************************************************
#pragma once

#include "mll_footmap.h"
#include "mll_map_section.h"
#include "stdint.h"

namespace mll {

enum class AlgorithmType : uint8_t {
    LEFT_HAND = 0,
    ADACHI,
    DIJKSTRA,
};

class MazeSolver {
   private:
    MazeSolver();

    // 歩数マップ
    Footmap footmap1;
    Footmap footmap2;
    uint8_t footmap_index;  // 現在有効な歩数マップのインデックス

    AlgorithmType algorithm;

    void updateFootmapAdachi(Footmap* fm, int8_t current_x = -1, int8_t current_y = -1);

    FirstPersonDirection getNextDirectionInSearchLeftHand(int8_t current_x, int8_t current_y, CardinalDirection current_angle);
    FirstPersonDirection getNextDirectionInSearchAdachi(int8_t current_x, int8_t current_y, CardinalDirection current_angle);

   public:
    // 最新の壁情報
    Map map;

    void setAlgorithm(AlgorithmType algorithm);

    // 歩数マップ更新
    // Activity 内から直接呼び出す想定
    // 割り込み周期とは同期せず実行する
    // 現在無効の歩数マップを更新し、終了時に有効な歩数マップを切り替える
    // 現在座標を渡すと、その座標に到達した時点で更新を打切る
    // 戻り値は更新が完了した歩数マップのインデックス
    uint8_t updateFootmap(int8_t current_x = -1, int8_t current_y = -1);

    // 歩数マップの初期化
    void clearFootmap();

    // 現在有効な歩数マップのポインタを取得
    Footmap* getFootmap();

    // 現在有効な歩数マップのインデックスを取得
    uint8_t getFootmapIndex();

    // 探索中、次に進むべき方向を返す
    // 目的地に到達している場合の戻り値は未定義
    FirstPersonDirection getNextDirectionInSearch(int8_t current_x, int8_t current_y, CardinalDirection current_angle);

    // 現在の動作の目的地座標
    MultiplePosition destination;

    static MazeSolver* getInstance();
};

}  // namespace mll
