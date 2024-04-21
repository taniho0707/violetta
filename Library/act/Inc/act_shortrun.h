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
    void init() override;
    Status run() override;
    void finalize() override;
};

}  // namespace act
