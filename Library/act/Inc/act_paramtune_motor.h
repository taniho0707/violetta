//******************************************************************************
// @addtogroup ACT
// @file       act_paramtune_motor.h
// @brief      ParamtuneMotor Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class ParamtuneMotorActivity : public IActivity {
   private:
    MotorParameterTuneType motor_tune_type;  // モーターパラメータ調整の種類
    bool motor_tune_right;                   // 右周りを調整するかどうか、falseなら左回り

   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
