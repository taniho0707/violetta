//******************************************************************************
// @addtogroup ACT
// @file       act_debug.h
// @brief      Debug Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

// MPL
#include "mpl_battery.h"
#include "mpl_debug.h"
#include "mpl_encoder.h"
#include "mpl_imu.h"
#include "mpl_led.h"
#include "mpl_motor.h"
#include "mpl_timer.h"
#include "mpl_wallsensor.h"

namespace act {

class DebugActivity : public IActivity {
   private:
   public:
    void init() override;
    Status run() override;
    void finalize() override;
};

}  // namespace act
