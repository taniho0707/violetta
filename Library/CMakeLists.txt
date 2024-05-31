cmake_minimum_required(VERSION 3.10)
add_subdirectory(act)
add_subdirectory(hal)
add_subdirectory(misc)
add_subdirectory(mll)
add_subdirectory(mpl)
add_subdirectory(msg)
add_subdirectory(cmd)

add_library(Library STATIC
    $<TARGET_OBJECTS:hal>
    $<TARGET_OBJECTS:mpl>
    $<TARGET_OBJECTS:act>
    $<TARGET_OBJECTS:msg>
    $<TARGET_OBJECTS:cmd>
    $<TARGET_OBJECTS:mll>
)
