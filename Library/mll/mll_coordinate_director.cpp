//******************************************************************************
// @addtogroup MLL
// @file       mll_coordinate_director.cpp
// @brief      マウスの区画移動計画を立て、区画情報を持つクラス
//******************************************************************************
#include "mll_coordinate_director.h"

#include "cmd_format.h"
#include "hal_conf.h"
#include "msg_format_wall_analyser.h"

using namespace mll;

CoordinateDirector::CoordinateDirector() {
    maze_solver = MazeSolver::getInstance();
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
        case mll::FirstPersonDirection::RIGHT:
            moves[0] = OperationMoveCombination{OperationMoveType::SLALOM90SML_RIGHT, 0.f};
            length = 1;
            current_section.move(OperationMoveType::SLALOM90SML_RIGHT);
            break;
        case FirstPersonDirection::BACK:
            moves[0] = OperationMoveCombination{OperationMoveType::TRAPACCEL_STOP, 45.f};
            moves[1] = OperationMoveCombination{OperationMoveType::PIVOTTURN, misc::PI};
            moves[2] = OperationMoveCombination{OperationMoveType::TRAPACCEL, 45.f};
            length = 3;
            current_section.move(OperationMoveType::PIVOTTURN);
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

CoordinateDirector* CoordinateDirector::getInstance() {
    static CoordinateDirector instance;
    return &instance;
}
