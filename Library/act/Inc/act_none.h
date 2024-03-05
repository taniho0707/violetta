//******************************************************************************
// @addtogroup ACT
// @file       act_none.h
// @brief      None Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class NoneActivity : public IActivity {
   private:
   public:
    void init() override;
    Status run() override;
    void finalize() override;
};

}  // namespace act
