//******************************************************************************
// @addtogroup ACT
// @file       act_enkaigei.h
// @brief      Enkaigei Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class EnkaigeiActivity : public IActivity {
   private:
   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
