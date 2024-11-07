//******************************************************************************
// @addtogroup ACT
// @file       act_system_identification.h
// @brief      SystemIdentification Activity
//******************************************************************************
#pragma once

#include "act_conf.h"

namespace act {

class SystemIdentificationActivity : public IActivity {
   private:
    SystemIdentificationType system_identification_type;

   public:
    void init(ActivityParameters &params) override;
    Status run() override;
    void finalize(ActivityParameters &params) override;
};

}  // namespace act
