//******************************************************************************
// @addtogroup MLL
// @file       mll_position_corrector.h
// @brief      マウスの物理座標を補正するクラス、今の所壁切れ
//******************************************************************************
#pragma once

namespace mll {

class PositionCorrector {
   private:
    PositionCorrector();

   public:
    // TODO: Implement

    void update();

    static PositionCorrector* getInstance();
};

}  // namespace mll
