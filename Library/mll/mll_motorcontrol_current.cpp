//******************************************************************************
// @addtogroup MLL
// @file       mll_motorcontrol_current.cpp
// @brief      モーターの電流フィードバック制御
//******************************************************************************
#include "mll_motorcontrol_current.h"

mll::MotorControlCurrent::MotorControlCurrent() {
}

mll::MotorControlResult mll::MotorControlCurrent::calc(MotorControlData& motor_control_data, MotorControlData& motor_control_data_target) {
    return mll::MotorControlResult::NO_IMPLEMENT;
}
