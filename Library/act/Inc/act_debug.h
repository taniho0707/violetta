//******************************************************************************
// @addtogroup ACT
// @file       act_debug.h
// @brief      Debug Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class DebugActivity : public IActivity {
   private:
   public:
    void init() override;
    Status run() override;
    void finalize() override;
};

}  // namespace act
