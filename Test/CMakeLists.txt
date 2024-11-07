cmake_minimum_required(VERSION 3.10)

add_custom_target(unittest)
enable_testing()

# プラットフォーム分岐
if(${PLATFORM} STREQUAL Linux)
    add_executable(test_mll_wall test_mll_wall.cpp)
    target_link_libraries(test_mll_wall Library)
    add_dependencies(unittest test_mll_wall)

    add_executable(test_mll_map_section test_mll_map_section.cpp)
    target_link_libraries(test_mll_map_section Library)
    add_dependencies(unittest test_mll_map_section)

    add_executable(test_mll_maze_solver test_mll_maze_solver.cpp)
    target_link_libraries(test_mll_maze_solver Library)
    add_dependencies(unittest test_mll_maze_solver)
elseif(${PLATFORM} STREQUAL STM32)
    message(FATAL_ERROR "[BUILD PLATFORM] -DTARGET_GROUP=Test -DPLATFORM=STM32 is not implemented")
else()
    message(FATAL_ERROR "[BUILD PLATFORM] Invalid or unspecified platform. Please specify it as -DPLATFORM=[Linux,STM32]")
endif()

add_test(NAME test_mll_wall COMMAND test_mll_wall)
add_test(NAME test_mll_map_section COMMAND test_mll_map_section)
add_test(NAME test_mll_maze_solver COMMAND test_mll_maze_solver)
