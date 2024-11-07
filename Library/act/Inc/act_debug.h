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
    DebugLogType log_type;

   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
