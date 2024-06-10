//******************************************************************************
// @addtogroup MLL
// @file       mll_wall.h
// @brief      壁データの定義
//******************************************************************************
#pragma once

#include "mll_position.h"
#include "stdint.h"

namespace mll {

/**
 * @brief 壁データを簡単に扱うためのクラス。
 */
struct Walldata {
    /**
     * @brief 右ビットから順に前，右，後、左のデータが格納されている
     */
    uint8_t data;

    /**
     * @brief =演算子のオーバーロード<br>char型の代入ができます。
     * @param input 代入したい変数
     */
    void operator=(Walldata input);

    void operator+=(Walldata input);

    void operator|=(Walldata input);

    /**
     * @brief 指定した方向に壁があるか返します。
     * @param dir 壁を調べたい方向
     * @return 壁があればtrue，なければfalse
     */
    bool isExistWall(FirstPersonDirection dir);

    /**
     * @brief 指定した方向に壁を追加します。
     * @param dir 壁を追加したい方向
     */
    void addWall(FirstPersonDirection dir);

    /**
     * @brief 指定した方向の壁を除去します。
     * @param dir 壁を除去したい方向
     */
    void removeWall(FirstPersonDirection dir);

    static Walldata rotateWallToAbsolute(Walldata wall, CardinalDirection angle) {
        Walldata newwall;
        newwall.data = 0;
        for (int i = 0; i < 4; i++) {
            if (wall.isExistWall((FirstPersonDirection)((i + (int)angle) % 4))) {
                newwall.addWall((FirstPersonDirection)i);
            }
        }
        return newwall;
    }

    static Walldata rotateWallToRelative(Walldata wall, CardinalDirection angle) {
        Walldata newwall;
        newwall.data = 0;
        for (int i = 0; i < 4; i++) {
            if (wall.isExistWall((FirstPersonDirection)((i - (int)angle + 4) % 4))) {
                newwall.addWall((FirstPersonDirection)i);
            }
        }
        return newwall;
    }

    // static Walldata Walldata::rotateWallToAbsolute(Walldata wall, MazeAngle angle) {
    //     Walldata ret;
    //     if (angle == MazeAngle::NORTH) return wall;
    //     else if (angle == MazeAngle::EAST) {
    //         if (wall.isExistWall(MouseAngle::FRONT)) ret.addWall(MouseAngle::RIGHT);
    //         if (wall.isExistWall(MouseAngle::RIGHT)) ret.addWall(MouseAngle::BACK);
    //         if (wall.isExistWall(MouseAngle::BACK)) ret.addWall(MouseAngle::LEFT);
    //         if (wall.isExistWall(MouseAngle::LEFT)) ret.addWall(MouseAngle::FRONT);
    //     } else if (angle == MazeAngle::SOUTH) {
    //         if (wall.isExistWall(MouseAngle::FRONT)) ret.addWall(MouseAngle::BACK);
    //         if (wall.isExistWall(MouseAngle::RIGHT)) ret.addWall(MouseAngle::LEFT);
    //         if (wall.isExistWall(MouseAngle::BACK)) ret.addWall(MouseAngle::FRONT);
    //         if (wall.isExistWall(MouseAngle::LEFT)) ret.addWall(MouseAngle::RIGHT);
    //     } else {
    //         if (wall.isExistWall(MouseAngle::FRONT)) ret.addWall(MouseAngle::LEFT);
    //         if (wall.isExistWall(MouseAngle::RIGHT)) ret.addWall(MouseAngle::FRONT);
    //         if (wall.isExistWall(MouseAngle::BACK)) ret.addWall(MouseAngle::RIGHT);
    //         if (wall.isExistWall(MouseAngle::LEFT)) ret.addWall(MouseAngle::BACK);
    //     }
    //     return ret;
    // }

    // static Walldata Walldata::rotateWallToRelative(Walldata wall, MazeAngle angle) {
    //     Walldata ret;
    //     if (angle == MazeAngle::NORTH) return wall;
    //     else if (angle == MazeAngle::EAST) {
    //         if (wall.isExistWall(MouseAngle::FRONT)) ret.addWall(MouseAngle::LEFT);
    //         if (wall.isExistWall(MouseAngle::RIGHT)) ret.addWall(MouseAngle::FRONT);
    //         if (wall.isExistWall(MouseAngle::BACK)) ret.addWall(MouseAngle::RIGHT);
    //         if (wall.isExistWall(MouseAngle::LEFT)) ret.addWall(MouseAngle::BACK);
    //     } else if (angle == MazeAngle::SOUTH) {
    //         if (wall.isExistWall(MouseAngle::FRONT)) ret.addWall(MouseAngle::BACK);
    //         if (wall.isExistWall(MouseAngle::RIGHT)) ret.addWall(MouseAngle::LEFT);
    //         if (wall.isExistWall(MouseAngle::BACK)) ret.addWall(MouseAngle::FRONT);
    //         if (wall.isExistWall(MouseAngle::LEFT)) ret.addWall(MouseAngle::RIGHT);
    //     } else {
    //         if (wall.isExistWall(MouseAngle::FRONT)) ret.addWall(MouseAngle::RIGHT);
    //         if (wall.isExistWall(MouseAngle::RIGHT)) ret.addWall(MouseAngle::BACK);
    //         if (wall.isExistWall(MouseAngle::BACK)) ret.addWall(MouseAngle::LEFT);
    //         if (wall.isExistWall(MouseAngle::LEFT)) ret.addWall(MouseAngle::FRONT);
    //     }
    //     return ret;
    // }
};

}  // namespace mll
