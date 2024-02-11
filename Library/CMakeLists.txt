cmake_minimum_required(VERSION 3.0)
add_subdirectory(act)
add_subdirectory(hal)
add_subdirectory(misc)
add_subdirectory(mll)
add_subdirectory(mpl)
add_subdirectory(msg)

add_library(Library STATIC
    $<TARGET_OBJECTS:hal>
    $<TARGET_OBJECTS:mpl>
)
