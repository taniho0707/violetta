//******************************************************************************
// @addtogroup MPL
// @file       mpl_ui.h
// @brief      UI統括制御
//******************************************************************************
#pragma once

#include "mpl_conf.h"

namespace mpl {

class Ui {
   private:
    Ui();

   public:
    mpl::MplStatus initPort();
    void deinitPort();

    void interruptPeriodic();

    static Ui* getInstance();
};

}  // namespace mpl

// MEMO
// - struct Command {
//     Type type
//     union {
//       struct LedParam {}
//       struct SpeakerParam {}
//     }
//   }
