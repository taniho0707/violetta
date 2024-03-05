//******************************************************************************
// @addtogroup MPL
// @file       mpl_encoder.h
// @brief      エンコーダ制御
//******************************************************************************
#pragma once

#include "hal_encoder.h"
#include "mpl_conf.h"

namespace mpl {

class Encoder {
   private:
    Encoder();

   public:
    void initPort();
    void deinitPort();

    mpl::MplStatus scanEncoderSync(hal::EncoderData& data);

    void interrupt();

    static Encoder* getInstance();
};

}  // namespace mpl
