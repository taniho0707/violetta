//******************************************************************************
// @addtogroup MLL
// @file       mll_footmap.h
// @brief      歩数マップを管理するクラス
//******************************************************************************
#pragma once

#include <array>

#include "mll_wall.h"

namespace mll {

class Footmap {
   private:
    std::array<std::array<uint16_t, 32>, 32> map;

   public:
    Footmap();

    /**
     * @brief 歩数マップをクリアします
     */
    void clear();

    /**
     * @brief 歩数マップを取得します
     * @param x 取得するx座標
     * @param y 取得するy座標
     * @param out 座標外の返り値
     * @return 設定した座標の歩数
     */
    uint16_t get(const int8_t x, const int8_t y, const uint16_t out);

    /**
     * @brief 歩数マップを設定します
     * @param x 設定するx座標
     * @param y 設定するy座標
     * @param data 設定する歩数
     */
    bool set(const int8_t x, const int8_t y, const uint16_t data);

    bool isOutside(const int8_t x, const int8_t y);

    // 指定座標の周囲の歩数の最小値を取得
    // ただし絶対座標系での壁情報を引数に取り、壁がある方向には歩数を返さない
    uint16_t getMinNextTo(const int8_t x, const int8_t y, Walldata wall);

    ~Footmap();
};

}  // namespace mll
