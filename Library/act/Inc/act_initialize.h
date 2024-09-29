//******************************************************************************
// @addtogroup ACT
// @file       act_initialize.h
// @brief      Initialize Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class InitializeActivity : public IActivity {
   private:
   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
