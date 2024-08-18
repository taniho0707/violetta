//******************************************************************************
// @addtogroup MLL
// @file       mll_position.h
// @brief      位置や向きに関する定義
//******************************************************************************
#pragma once

#include "mll_operation_move_type.h"
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

// マウスの物理位置を定義
struct MousePhysicalPosition {
    float x;
    float y;
    float angle;
};

struct MouseVelocity {
    float translation;
    float rotation;
};

// マウスの論理位置を定義
struct MouseSectionPosition {
    int16_t x;
    int16_t y;
    CardinalDirection d;

    // マウスを1歩進める
    // NOTE: 直進方向は1区画固定移動である点に注意
    void move(mll::OperationMoveType type) {
        switch (type) {
            case mll::OperationMoveType::SLALOM90SML_RIGHT:
                if (d == CardinalDirection::NORTH) {
                    ++x;
                    d = CardinalDirection::EAST;
                } else if (d == CardinalDirection::EAST) {
                    --y;
                    d = CardinalDirection::SOUTH;
                } else if (d == CardinalDirection::SOUTH) {
                    --x;
                    d = CardinalDirection::WEST;
                } else if (d == CardinalDirection::WEST) {
                    ++y;
                    d = CardinalDirection::NORTH;
                }
                break;
            case mll::OperationMoveType::SLALOM90SML_LEFT:
                if (d == CardinalDirection::NORTH) {
                    --x;
                    d = CardinalDirection::WEST;
                } else if (d == CardinalDirection::EAST) {
                    ++y;
                    d = CardinalDirection::NORTH;
                } else if (d == CardinalDirection::SOUTH) {
                    ++x;
                    d = CardinalDirection::EAST;
                } else if (d == CardinalDirection::WEST) {
                    --y;
                    d = CardinalDirection::SOUTH;
                }
                break;
            // TODO: 最短用ターンの実装
            case mll::OperationMoveType::SLALOM90_RIGHT:
            case mll::OperationMoveType::SLALOM90_LEFT:
            case mll::OperationMoveType::SLALOM180_RIGHT:
            case mll::OperationMoveType::SLALOM180_LEFT:
            case mll::OperationMoveType::SLALOM45IN_RIGHT:
            case mll::OperationMoveType::SLALOM45IN_LEFT:
            case mll::OperationMoveType::SLALOM45OUT_RIGHT:
            case mll::OperationMoveType::SLALOM45OUT_LEFT:
            case mll::OperationMoveType::SLALOM135IN_RIGHT:
            case mll::OperationMoveType::SLALOM135IN_LEFT:
            case mll::OperationMoveType::SLALOM135OUT_RIGHT:
            case mll::OperationMoveType::SLALOM135OUT_LEFT:
            case mll::OperationMoveType::SLALOM90OBL_RIGHT:
            case mll::OperationMoveType::SLALOM90OBL_LEFT:
                break;
            case mll::OperationMoveType::TRAPACCEL:
            case mll::OperationMoveType::TRAPACCEL_STOP:
                if (d == CardinalDirection::NORTH) {
                    ++y;
                } else if (d == CardinalDirection::EAST) {
                    ++x;
                } else if (d == CardinalDirection::SOUTH) {
                    --y;
                } else if (d == CardinalDirection::WEST) {
                    --x;
                }
                break;
            case mll::OperationMoveType::PIVOTTURN:
                if (d == CardinalDirection::NORTH) {
                    --y;
                    d = CardinalDirection::SOUTH;
                } else if (d == CardinalDirection::EAST) {
                    --x;
                    d = CardinalDirection::WEST;
                } else if (d == CardinalDirection::SOUTH) {
                    ++y;
                    d = CardinalDirection::NORTH;
                } else if (d == CardinalDirection::WEST) {
                    ++x;
                    d = CardinalDirection::EAST;
                }
                break;
            // TODO: 斜め用直進の実装
            // case mll::OperationMoveType::TRAPDIAGO:
            //     break;
            // case mll::OperationMoveType::EXTRALENGTH:
            case mll::OperationMoveType::WAIT:
            case mll::OperationMoveType::LENGTH:
            case mll::OperationMoveType::UNDEFINED:
            case mll::OperationMoveType::STOP:
                break;
        }
    }
};

class MultiplePosition {
   private:
    misc::Point<uint16_t> curs[MAX_LENGTH_MULTIPLE_POSITION];
    uint32_t stored;

   public:
    void clear();
    bool isInclude(misc::Point<uint16_t> cur);

    // 指定したインデックスにデータが含まれているかを返す
    bool hasData(uint8_t);

    bool add(int16_t x, int16_t y);
    void remove(int16_t x, int16_t y);
    uint8_t length();

    // for-range 用メソッド
    // NOTE: 必ず一つ以上のデータが入っているとする
    misc::Point<uint16_t>* begin();
    misc::Point<uint16_t>* end();

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
