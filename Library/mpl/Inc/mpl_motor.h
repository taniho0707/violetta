//******************************************************************************
// @addtogroup MPL
// @file       mpl_motor.h
// @brief      モータ制御
//******************************************************************************
#pragma once

#include "hal_motor.h"
#include "mpl_conf.h"
#include "stdint.h"

namespace mpl {

class Motor {
   private:
    Motor();

   public:
    void initPort();
    void deinitPort();

    mpl::MplStatus setFloat();

    mpl::MplStatus setDutyL(float duty);
    mpl::MplStatus setDutyR(float duty);
    mpl::MplStatus setDuty(float duty_l, float duty_r);

    void interrupt();

    static Motor* getInstance();
};

}  // namespace mpl
