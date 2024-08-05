#include "acutest.h"
#include "mll_wall.h"

void test_wall_set_and_get() {
    mll::Walldata wall;

    TEST_CHECK(!wall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(!wall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(!wall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(!wall.isExistWall(mll::FirstPersonDirection::LEFT));

    wall.addWall(mll::FirstPersonDirection::FRONT);
    wall.addWall(mll::FirstPersonDirection::RIGHT);
    wall.addWall(mll::FirstPersonDirection::BACK);
    wall.addWall(mll::FirstPersonDirection::LEFT);

    TEST_CHECK(wall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(wall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(wall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(wall.isExistWall(mll::FirstPersonDirection::LEFT));
}

void test_wall_remove() {
    mll::Walldata wall;

    wall.addWall(mll::FirstPersonDirection::FRONT);
    wall.addWall(mll::FirstPersonDirection::RIGHT);
    wall.addWall(mll::FirstPersonDirection::BACK);
    wall.addWall(mll::FirstPersonDirection::LEFT);

    wall.removeWall(mll::FirstPersonDirection::FRONT);
    wall.removeWall(mll::FirstPersonDirection::RIGHT);
    wall.removeWall(mll::FirstPersonDirection::BACK);
    wall.removeWall(mll::FirstPersonDirection::LEFT);

    TEST_CHECK(!wall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(!wall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(!wall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(!wall.isExistWall(mll::FirstPersonDirection::LEFT));
}

void test_wall_rotate_to_absolute() {
    mll::Walldata wall;

    wall.addWall(mll::FirstPersonDirection::FRONT);
    wall.addWall(mll::FirstPersonDirection::LEFT);

    mll::Walldata newwall = mll::Walldata::rotateWallToAbsolute(wall, mll::CardinalDirection::NORTH);
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::LEFT));

    newwall = mll::Walldata::rotateWallToAbsolute(wall, mll::CardinalDirection::EAST);
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::LEFT));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::FRONT));

    newwall = mll::Walldata::rotateWallToAbsolute(wall, mll::CardinalDirection::SOUTH);
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::LEFT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::RIGHT));

    newwall = mll::Walldata::rotateWallToAbsolute(wall, mll::CardinalDirection::WEST);
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::LEFT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::BACK));
}

void test_wall_rotate_to_relative() {
    mll::Walldata wall;

    wall.addWall(mll::FirstPersonDirection::FRONT);
    wall.addWall(mll::FirstPersonDirection::LEFT);

    mll::Walldata newwall = mll::Walldata::rotateWallToRelative(wall, mll::CardinalDirection::NORTH);
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::LEFT));

    newwall = mll::Walldata::rotateWallToRelative(wall, mll::CardinalDirection::EAST);
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::LEFT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::FRONT));

    newwall = mll::Walldata::rotateWallToRelative(wall, mll::CardinalDirection::SOUTH);
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::BACK));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::LEFT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::RIGHT));

    newwall = mll::Walldata::rotateWallToRelative(wall, mll::CardinalDirection::WEST);
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::LEFT));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::FRONT));
    TEST_CHECK(newwall.isExistWall(mll::FirstPersonDirection::RIGHT));
    TEST_CHECK(!newwall.isExistWall(mll::FirstPersonDirection::BACK));
}

TEST_LIST = {
    {       "test_wall_set_and_get",        test_wall_set_and_get},
    {            "test_wall_remove",             test_wall_remove},
    {"test_wall_rotate_to_absolute", test_wall_rotate_to_absolute},
    {"test_wall_rotate_to_relative", test_wall_rotate_to_relative},
    {                          NULL,                         NULL}
};
