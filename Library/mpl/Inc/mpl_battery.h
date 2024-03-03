//******************************************************************************
// @addtogroup MPL
// @file       mpl_battery.h
// @brief      バッテリー電圧制御
//******************************************************************************
#pragma once

#include "hal_battery.h"
#include "mpl_conf.h"

namespace mpl {

class Battery {
   private:
    Battery();

   public:
    mpl::MplStatus initPort();
    void deinitPort();

    mpl::MplStatus scanSync(float& voltage);

    void interrupt();

    static Battery* getInstance();
};

}  // namespace mpl
