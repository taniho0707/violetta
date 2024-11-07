//******************************************************************************
// @addtogroup ACT
// @file       act_standby.h
// @brief      Standby Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class StandbyActivity : public IActivity {
   private:
   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
