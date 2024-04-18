cmake_minimum_required(VERSION 3.10)

if(${MOUSE} STREQUAL Violetta)
    add_subdirectory(Violetta)
elseif(${MOUSE} STREQUAL Lazuli)
    add_subdirectory(Lazuli)
elseif(${MOUSE} STREQUAL Zirconia2kai)
    add_subdirectory(zirconia2kai)
endif()
