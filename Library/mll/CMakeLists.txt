cmake_minimum_required(VERSION 3.10)
add_library(mll STATIC
    mll_motorcontrol_current.cpp
    mll_logger.cpp
    mll_localizer.cpp
    mll_position.cpp
    mll_wall.cpp
    mll_map_section.cpp
    mll_motor_controller.cpp
    mll_operation_controller.cpp
    mll_trajectory.cpp
    mll_wall_analyser.cpp
    mll_footmap.cpp
    mll_maze_solver.cpp
)
