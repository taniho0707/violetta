//******************************************************************************
// @addtogroup MLL
// @file       mll_crash_detector.h
// @brief      マウスの衝突判定を行うクラス
//******************************************************************************
#pragma once

#include "mll_position.h"
#include "stdint.h"

namespace mll {

class CrashDetector {
   private:
    CrashDetector();

   public:
    // 最後の衝突開始時刻を返す [us]
    uint32_t getLastCrashStartTime();

    // 最後の衝突開始物理座標を返す
    MousePhysicalPosition getLastCrashStartPosition();

    // 最後の衝突開始からの推定物理移動距離を返す
    MousePhysicalPosition getGuessDistanceFromLastCrash();

    static CrashDetector* getInstance();
};

}  // namespace mll
