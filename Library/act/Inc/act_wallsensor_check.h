//******************************************************************************
// @addtogroup ACT
// @file       act_wallsensor_check.h
// @brief      Wallsensor Check Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class WallsensorCheckActivity : public IActivity {
   private:
   public:
    void init() override;
    Status run() override;
    void finalize() override;
};

}  // namespace act
