#include "acutest.h"
#include "mll_maze_solver.h"

void test_set_algorithm() {
    mll::AlgorithmType algorithm;
}

void test_update_footmap() {}

void test_get_footmap() {}

void test_get_footmap_index() {}

void test_get_next_direction_in_search() {}

TEST_LIST = {
    {               "test_set_algorithm",                test_set_algorithm},
    {              "test_update_footmap",               test_update_footmap},
    {                 "test_get_footmap",                  test_get_footmap},
    {           "test_get_footmap_index",            test_get_footmap_index},
    {"test_get_next_direction_in_search", test_get_next_direction_in_search},
    {                               NULL,                              NULL}
};

// void test_map_goal() {
//     mll::Map map;
//     map.addGoal(1, 2);
//     map.addGoal(10, 9);

//     TEST_CHECK(map.goals.length() == 2);
//     TEST_CHECK(map.goals.isInclude(misc::Point<uint16_t>{1, 2}));
//     TEST_CHECK(map.goals.isInclude(misc::Point<uint16_t>{10, 9}));
// }
