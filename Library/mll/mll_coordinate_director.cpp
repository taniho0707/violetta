//******************************************************************************
// @addtogroup MLL
// @file       mll_coordinate_director.cpp
// @brief      マウスの区画移動計画を立て、区画情報を持つクラス
//******************************************************************************
#include "mll_coordinate_director.h"

#include "cmd_format.h"
#include "hal_conf.h"
#include "mll_logger.h"
#include "msg_format_wall_analyser.h"

// for Debug
#include "mpl_timer.h"
#include "msg_format_localizer.h"

using namespace mll;

CoordinateDirector::CoordinateDirector() {
    current_section = MouseSectionPosition{0, 1, CardinalDirection::NORTH};
    msg_server = msg::MessageServer::getInstance();
    solver = mll::MazeSolver::getInstance();
    solver->clearFootmap();
    solver->map.format();
}

// マウスが次に行うべき動作 (MoveType) を返す
// この関数は、Localizer と OperationCoordinator からの情報をもとに判断する
void CoordinateDirector::getNextMove(OperationMoveCombination* moves, uint16_t& length) {
    msg::MsgFormatWallAnalyser msg_wall_analyzer;
    msg_server->receiveMessage(msg::ModuleId::WALLANALYSER, &msg_wall_analyzer);
    solver->map.setWall(current_section.x, current_section.y, current_section.d, msg_wall_analyzer.front_wall);
    solver->map.setReached(current_section.x, current_section.y);

    // TODO: LED Indicator with DMA
    // // LED Indicator
    // cmd::CommandFormatUiOut cmd_ui_out;
    // if (msg_wall_analyzer.front_wall.isExistWall(FirstPersonDirection::LEFT)) {
    //     cmd_ui_out.type = UiOutputEffect::WALL_EXIST_LEFT;
    //     msg_server->sendMessage(msg::ModuleId::UI, &cmd_ui_out);
    // }
    // if (msg_wall_analyzer.front_wall.isExistWall(FirstPersonDirection::RIGHT)) {
    //     cmd_ui_out.type = UiOutputEffect::WALL_EXIST_RIGHT;
    //     msg_server->sendMessage(msg::ModuleId::UI, &cmd_ui_out);
    // }
    // if (msg_wall_analyzer.front_wall.isExistWall(FirstPersonDirection::FRONT)) {
    //     cmd_ui_out.type = UiOutputEffect::WALL_EXIST_FRONT;
    //     msg_server->sendMessage(msg::ModuleId::UI, &cmd_ui_out);
    // }

    // mll::LogFormatSearch log = {mpl::Timer::getMicroTime()};
    // msg::MsgFormatLocalizer msg_localizer;
    // msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
    // log.current_position = MousePhysicalPosition{msg_localizer.position_x, msg_localizer.position_y, msg_localizer.position_theta};
    // log.current_section = current_section;
    // log.current_walldata = msg_wall_analyzer.front_wall;
    // Logger::getInstance()->save(mll::LogType::SEARCH, &log);

    auto next_direction = solver->getNextDirectionInSearch(current_section.x, current_section.y, current_section.d);
    switch (next_direction) {
        case FirstPersonDirection::FRONT:
            moves[0] = OperationMoveCombination{OperationMoveType::TRAPACCEL, 90.f};
            length = 1;
            current_section.move(OperationMoveType::TRAPACCEL);
            break;
        case FirstPersonDirection::LEFT:
            moves[0] = OperationMoveCombination{OperationMoveType::SLALOM90SML_LEFT, 0.f};
            length = 1;
            current_section.move(OperationMoveType::SLALOM90SML_LEFT);
            break;
        case FirstPersonDirection::RIGHT:
            moves[0] = OperationMoveCombination{OperationMoveType::SLALOM90SML_RIGHT, 0.f};
            length = 1;
            current_section.move(OperationMoveType::SLALOM90SML_RIGHT);
            break;
        case FirstPersonDirection::BACK:
            uint8_t i = 0;
            moves[i++] = OperationMoveCombination{OperationMoveType::TRAPACCEL_STOP, 45.f};
            if (msg_wall_analyzer.front_wall.isExistWall(FirstPersonDirection::FRONT)) {
                moves[i++] = OperationMoveCombination{OperationMoveType::CORRECTION_FRONT, 0.f};
                moves[i++] = OperationMoveCombination{OperationMoveType::WAIT, 500};  // NOTE: for DEBUG
            }
            if (msg_wall_analyzer.front_wall.isExistWall(FirstPersonDirection::LEFT)) {
                moves[i++] = OperationMoveCombination{OperationMoveType::PIVOTTURN_LEFT, misc::PI / 2};
                moves[i++] = OperationMoveCombination{OperationMoveType::CORRECTION_FRONT, 0.f};
                moves[i++] = OperationMoveCombination{OperationMoveType::WAIT, 500};  // NOTE: for DEBUG
                moves[i++] = OperationMoveCombination{OperationMoveType::PIVOTTURN_LEFT, misc::PI / 2};
            } else if (msg_wall_analyzer.front_wall.isExistWall(FirstPersonDirection::RIGHT)) {
                moves[i++] = OperationMoveCombination{OperationMoveType::PIVOTTURN_RIGHT, misc::PI / 2};
                moves[i++] = OperationMoveCombination{OperationMoveType::CORRECTION_FRONT, 0.f};
                moves[i++] = OperationMoveCombination{OperationMoveType::WAIT, 500};  // NOTE: for DEBUG
                moves[i++] = OperationMoveCombination{OperationMoveType::PIVOTTURN_RIGHT, misc::PI / 2};
            } else {
                moves[i++] = OperationMoveCombination{OperationMoveType::PIVOTTURN_LEFT, misc::PI};
            }
            moves[i++] = OperationMoveCombination{OperationMoveType::WAIT, 500};  // NOTE: for DEBUG
            moves[i++] = OperationMoveCombination{OperationMoveType::TRAPACCEL, 45.f};
            length = i;
            current_section.move(OperationMoveType::PIVOTTURN_LEFT);
            break;
    }
    return;
}

void CoordinateDirector::setAlgorithm(AlgorithmType type) {
    algorithm_type = type;
    solver->setAlgorithm(algorithm_type);
}

bool CoordinateDirector::isEnd() {
    // msg::MsgFormatLocalizer msg_localizer;
    // msg_server->receiveMessage(msg::ModuleId::LOCALIZER, &msg_localizer);
    if (current_section.x == target_pos.x && current_section.y == target_pos.y) {
        return true;
    } else {
        return false;
    }
}

const MouseSectionPosition CoordinateDirector::getCurrentSection() const {
    return current_section;
}

void CoordinateDirector::setTargetSection(MultiplePosition& pos) {
    target_all_pos.clear();
    if (pos.length() > 0) {
        for (auto it = pos.begin(), e = pos.end(); it != e; ++it) {
            target_all_pos.add(it->x, it->y);
            solver->map.addGoal(it->x, it->y);
            target_pos.x = it->x;
            target_pos.y = it->y;  // FIXME: 適切な値に修正する
        }
    }
}

// マウスの現在の目標区画情報を返す
const misc::Point<uint16_t> CoordinateDirector::getTargetSection() const {
    return target_pos;
}

const uint16_t CoordinateDirector::debugOutput(char* buf, uint8_t n) const {
    return solver->string(buf, n);
}

CoordinateDirector* CoordinateDirector::getInstance() {
    static CoordinateDirector instance;
    return &instance;
}
