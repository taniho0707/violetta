cmake_minimum_required(VERSION 3.0)

add_library(GitHash STATIC
    Githash.cpp
    githash.h
)

add_custom_command(
    OUTPUT githash.h
    COMMAND "sh" "${CMAKE_SOURCE_DIR}/Script/githash.sh" ">" "${CMAKE_SOURCE_DIR}/githash.h"
)

add_custom_command(
    TARGET GitHash
    POST_BUILD
    COMMAND "rm" "${CMAKE_SOURCE_DIR}/githash.h"
)
