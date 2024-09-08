//******************************************************************************
// @addtogroup ACT
// @file       act_wallsensor_run.h
// @brief      Wallsensor Run Activity for LazuliSensor
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class WallsensorRunActivity : public IActivity {
   private:
   public:
    void init() override;
    Status run() override;
    void finalize() override;
};

}  // namespace act