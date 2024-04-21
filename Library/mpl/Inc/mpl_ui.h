//******************************************************************************
// @addtogroup MPL
// @file       mpl_ui.h
// @brief      UI統括制御
//******************************************************************************
#pragma once

namespace mpl {

class Ui {
   private:
    Ui();

   public:
    void initPort();
    void deinitPort();

    void interrupt();

    static Ui* getInstance();
};

}  // namespace mpl
