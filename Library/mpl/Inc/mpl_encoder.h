//******************************************************************************
// @addtogroup MPL
// @file       mpl_encoder.h
// @brief      エンコーダ制御
//******************************************************************************
#pragma once

#include "hal_encoder.h"
#include "mpl_conf.h"
#include "msg_format_encoder.h"

namespace mpl {

class Encoder {
   private:
    Encoder();

    hal::EncoderData last;
    msg::MsgFormatEncoder msg_format;

   public:
    void initPort();
    void deinitPort();

    mpl::MplStatus scanEncoderSync(hal::EncoderData& data);

    void interruptPeriodic();

    static Encoder* getInstance();
};

}  // namespace mpl
