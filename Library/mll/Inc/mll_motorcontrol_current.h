//******************************************************************************
// @addtogroup MLL
// @file       mll_motorcontrol_current.h
// @brief      モータの電流制御ロジック
//******************************************************************************
#pragma once

#include "mll_conf_motor.h"

namespace mll {

class MotorControlCurrent {
   private:
    MotorControlPID pid;

   public:
    MotorControlCurrent();

    // MotorControlData を参照で受取り、実際のバッテリー電圧と実際のモーター電流値、
    // 前回入力したDutyと前回の目標モーター電流値から、
    // 今回入力すべきDutyを設定する calc 関数
    mll::MotorControlResult calc(MotorControlData& motor_control_data, MotorControlData& motor_control_data_target);
};

}  // namespace mll
