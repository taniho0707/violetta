//******************************************************************************
// @addtogroup MPL
// @file       mpl_ui.cpp
// @brief      UI統括制御
//******************************************************************************
#include "mpl_ui.h"

mpl::Ui::Ui() {}

mpl::MplStatus mpl::Ui::initPort() {
    // auto status = hal::initLedPort(num);
    // if (status != hal::HalStatus::SUCCESS) {
    //     return mpl::MplStatus::ERROR;
    // } else {
    //     return mpl::MplStatus::SUCCESS;
    // }
}

void mpl::Ui::deinitPort() {
}

void mpl::Ui::interruptPeriodic() {
}

mpl::Ui* mpl::Ui::getInstance() {
    static mpl::Ui instance;
    return &instance;
}
