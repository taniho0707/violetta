//******************************************************************************
// @addtogroup  Message
// @file        msg_format_wall_analyser.cpp
// @brief       Message Format Wallsensor Analyser
//******************************************************************************

#include "msg_format_wall_analyser.h"

#include "mpl_timer.h"
#include "msg_format.h"

using namespace msg;

msg::MsgFormatWallAnalyser::MsgFormatWallAnalyser() : MsgFormat(ModuleId::WALLANALYSER) {}

void MsgFormatWallAnalyser::copy(void* target) {
    auto* t = static_cast<MsgFormatWallAnalyser*>(target);
    t->moduleid = moduleid;
    t->count = count;
    t->time = time;
    t->front_wall = front_wall;
    t->distance_from_center = distance_from_center;
    t->distance_from_front = distance_from_front;
    t->angle_from_front = angle_from_front;
    t->kabekire_left = kabekire_left;
    t->kabekire_right = kabekire_right;
}

void MsgFormatWallAnalyser::update(void* from) {
    count++;
    time = mpl::Timer::getMicroTime();
    auto* f = static_cast<MsgFormatWallAnalyser*>(from);
    moduleid = f->moduleid;
    count = f->count;
    time = f->time;
    front_wall = f->front_wall;
    distance_from_center = f->distance_from_center;
    distance_from_front = f->distance_from_front;
    angle_from_front = f->angle_from_front;
    kabekire_left = f->kabekire_left;
    kabekire_right = f->kabekire_right;
}