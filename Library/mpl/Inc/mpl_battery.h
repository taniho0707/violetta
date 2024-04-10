//******************************************************************************
// @addtogroup MPL
// @file       mpl_battery.h
// @brief      バッテリー電圧制御
//******************************************************************************
#pragma once

#include "hal_battery.h"
#include "mpl_conf.h"
#include "msg_format_battery.h"

namespace mpl {

class Battery {
   private:
    Battery();

    float last;
    msg::MsgFormatBattery msg_format;

   public:
    mpl::MplStatus initPort();
    void deinitPort();

    mpl::MplStatus scanSync(float& voltage);

    void interruptPeriodic();

    static Battery* getInstance();
};

}  // namespace mpl
