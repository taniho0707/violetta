cmake_minimum_required(VERSION 3.0)
project("violetta")
include_directories(.
    Project/taniho/Inc
    Platform/Linux/Inc
    Library/act/Inc
    Library/hal/Inc
    Library/misc/Inc
    Library/mll/Inc
    Library/mpl/Inc
    Library/msg/Inc
)
add_definitions(-DLINUX
    -DMOUSE_VIOLETTA
    -D_USE_MATH_DEFINES
)
add_compile_options(-Wall
    -fmessage-length=0
    -fexceptions
    -funsigned-char
    -fpermissive
    -std=c++17
    -Wno-narrowing
    -g
    -gdwarf-2
)

add_subdirectory(Library)
add_subdirectory(Platform)
add_subdirectory(Project)
add_subdirectory(Tool)

add_executable(debug_taniho
    Project/taniho/taniho_linux.cpp
)

# add_custom_command(
#     OUTPUT githash.h
#     PRE_BUILD
#     COMMAND "sh" "${CMAKE_SOURCE_DIR}/Script/githash.sh" ">" "${CMAKE_SOURCE_DIR}/githash.h"
# )

# add_custom_command(
#     TARGET debug_taniho
#     POST_BUILD
#     COMMAND "rm" "${CMAKE_SOURCE_DIR}/githash.h"
# )

target_link_libraries(debug_taniho
    # act
    # hal
    # misc
    # mll
    mpl
    # msg
    # Linux
    taniho
    GitHash
)
