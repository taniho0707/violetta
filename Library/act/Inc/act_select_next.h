//******************************************************************************
// @addtogroup ACT
// @file       act_select_next.h
// @brief      SelectNext Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class SelectNextActivity : public IActivity {
   private:
   public:
    void init() override;
    Status run() override;
    void finalize() override;
};

}  // namespace act
