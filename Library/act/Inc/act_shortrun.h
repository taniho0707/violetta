//******************************************************************************
// @addtogroup ACT
// @file       act_shortrun.h
// @brief      Shortrun Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class ShortrunActivity : public IActivity {
   private:
   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
