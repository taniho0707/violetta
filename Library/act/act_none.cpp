//******************************************************************************
// @addtogroup ACT
// @file       act_none.cpp
// @brief      None Activity
//******************************************************************************
#include "act_none.h"

using namespace act;

void NoneActivity::init(ActivityParameters &params) {}

Status NoneActivity::run() {
    while (true) {
        // 何もしない
    }
    return Status::ERROR;
}

void NoneActivity::finalize(ActivityParameters &params) {}
