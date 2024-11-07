#include "acutest.h"
#include "mll_maze_solver.h"

void test_set_algorithm() {
    mll::AlgorithmType algorithm;
}

void test_update_footmap() {}

void test_get_footmap() {}

void test_get_footmap_index() {}

void test_get_next_direction_in_search_lefthand() {
    mll::MazeSolver* solver = mll::MazeSolver::getInstance();
    solver->setAlgorithm(mll::AlgorithmType::LEFT_HAND);

    // NO ADDITIONAL WALL
    mll::FirstPersonDirection direction = solver->getNextDirectionInSearch(0, 1, mll::CardinalDirection::NORTH);
    TEST_CHECK(direction == mll::FirstPersonDirection::FRONT);
    direction = solver->getNextDirectionInSearch(0, 1, mll::CardinalDirection::EAST);
    TEST_CHECK(direction == mll::FirstPersonDirection::LEFT);
    direction = solver->getNextDirectionInSearch(0, 1, mll::CardinalDirection::SOUTH);
    TEST_CHECK(direction == mll::FirstPersonDirection::LEFT);
    direction = solver->getNextDirectionInSearch(0, 1, mll::CardinalDirection::WEST);
    TEST_CHECK(direction == mll::FirstPersonDirection::LEFT);

    // ADDITIONAL WALL
    mll::Walldata walldata;
    walldata.addWall(mll::FirstPersonDirection::FRONT);
    walldata.addWall(mll::FirstPersonDirection::RIGHT);
    walldata.addWall(mll::FirstPersonDirection::LEFT);
    solver->map.setWall(0, 1, mll::CardinalDirection::NORTH, walldata);

    direction = solver->getNextDirectionInSearch(0, 1, mll::CardinalDirection::NORTH);
    TEST_CHECK(direction == mll::FirstPersonDirection::BACK);

    mll::Walldata walldata2;
    walldata2.addWall(mll::FirstPersonDirection::FRONT);
    walldata2.addWall(mll::FirstPersonDirection::LEFT);
    solver->map.setWall(0, 1, mll::CardinalDirection::NORTH, walldata2);

    direction = solver->getNextDirectionInSearch(0, 1, mll::CardinalDirection::NORTH);
    TEST_CHECK(direction == mll::FirstPersonDirection::RIGHT);
}

TEST_LIST = {
    {                        "test_set_algorithm",                         test_set_algorithm},
    {                       "test_update_footmap",                        test_update_footmap},
    {                          "test_get_footmap",                           test_get_footmap},
    {                    "test_get_footmap_index",                     test_get_footmap_index},
    {"test_get_next_direction_in_search_lefthand", test_get_next_direction_in_search_lefthand},
    {                                        NULL,                                       NULL}
};

// void test_map_goal() {
//     mll::Map map;
//     map.addGoal(1, 2);
//     map.addGoal(10, 9);

//     TEST_CHECK(map.goals.length() == 2);
//     TEST_CHECK(map.goals.isInclude(misc::Point<uint16_t>{1, 2}));
//     TEST_CHECK(map.goals.isInclude(misc::Point<uint16_t>{10, 9}));
// }
