//******************************************************************************
// @addtogroup ACT
// @file       act_module_test.h
// @brief      ModuleTest Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class ModuleTestActivity : public IActivity {
   private:
   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
