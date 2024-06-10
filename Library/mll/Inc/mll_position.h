//******************************************************************************
// @addtogroup MLL
// @file       mll_position.h
// @brief      位置や向きに関する定義
//******************************************************************************
#pragma once

#include "stdint.h"
#include "util.h"

namespace mll {

// MuitiplePositionクラスで使うバッファ長
constexpr uint8_t MAX_LENGTH_MULTIPLE_POSITION = 16;

// 4方位を定義
enum class CardinalDirection : uint8_t {
    NORTH,
    EAST,
    SOUTH,
    WEST,
};

// 8方位を定義
enum class IntercardinalDirectioin : uint8_t {
    NORTH,
    NORTHEAST,
    EAST,
    SOUTHEAST,
    SOUTH,
    SOUTHWEST,
    WEST,
    NORTHWEST,
};

// マウスから見たときの前後左右4方向を定義
enum class FirstPersonDirection : uint8_t {
    FRONT,
    RIGHT,
    BACK,
    LEFT,
};

class MultiplePosition {
   private:
    misc::Point<uint16_t> curs[MAX_LENGTH_MULTIPLE_POSITION];
    uint32_t stored;

   public:
    void clear();
    bool isInclude(misc::Point<uint16_t> cur);
    bool add(int16_t x, int16_t y);
    void remove(int16_t x, int16_t y);
    uint8_t length();

    MultiplePosition();
};

// TODO: 必要であれば実装する、マウスの座標を表記するクラス
// class Position {
//    private:
//     int16_t cur_x;
//     int16_t cur_y;

//     /// @todo 撤廃する
//     MazeAngle cur_angle;

//     MouseDirection cur_dir;

//    public:
//     void setPosition(int8_t x, int8_t y);
//     void setPosition(int8_t x, int8_t y, MazeAngle angle);
//     void setAngle(MazeAngle angle);
//     void setAngle(MouseDirection dir);

//     void setNextPosition(slalomparams::RunType type);

//     int16_t getPositionX();
//     int16_t getPositionY();
//     std::pair<int16_t, int16_t> getPosition();

//     MazeAngle getAngle();

//     // 斜め中ならtrue
//     bool isDiago();

//     static MazeAngle cnvMouseDirectionToMazeAngle(MouseDirection);
//     static MouseDirection cnvMazeAngleToMouseDirection(MazeAngle);

//     Position();
//     ~Position();
// };

}  // namespace mll
