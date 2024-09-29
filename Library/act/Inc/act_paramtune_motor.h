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
   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
