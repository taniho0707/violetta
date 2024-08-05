#include "acutest.h"
#include "mll_map_section.h"

void test_map_goal() {
    mll::Map map;
    map.addGoal(1, 2);
    map.addGoal(10, 9);

    TEST_CHECK(map.goals.length() == 2);
    TEST_CHECK(map.goals.isInclude(misc::Point<uint16_t>{1, 2}));
    TEST_CHECK(map.goals.isInclude(misc::Point<uint16_t>{10, 9}));
}

TEST_LIST = {
    {"test_map_goal", test_map_goal},
    {           NULL,          NULL}
};
