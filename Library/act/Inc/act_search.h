//******************************************************************************
// @addtogroup ACT
// @file       act_search.h
// @brief      Search Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class SearchActivity : public IActivity {
   private:
   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
