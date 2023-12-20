cmake_minimum_required(VERSION 3.4)

project("violetta")

# プラットフォーム分岐
if(${PLATFORM} STREQUAL Linux)
    message("[BUILD PLATFORM] Linux")
    include_directories(Platform/Linux/Inc)
    add_definitions(-DLINUX)
elseif(${PLATFORM} STREQUAL STM32)
    message("[BUILD PLATFORM] STM32")
    include_directories(Platform/STM32/Inc)
    add_definitions(-DSTM32)
else()
    message(FATAL_ERROR "[BUILD PLATFORM] Invalid or unspecified platform. Please specify it as -DPLATFORM=[Linux,STM32]")
endif()

########################
### General Settings ###
########################

include_directories(.
    Project/taniho/Inc
    Library/act/Inc
    Library/hal/Inc
    Library/misc/Inc
    Library/mll/Inc
    Library/mpl/Inc
    Library/msg/Inc
)
add_definitions(
    -DMOUSE_VIOLETTA
    -D_USE_MATH_DEFINES
)
add_compile_options(
    -Wall
    -fmessage-length=0
    -fexceptions
    -funsigned-char
    -fpermissive
    -std=c++17
    -Wno-narrowing
    -g
    -gdwarf-2
    -fPIC
)

add_subdirectory(Library)
add_subdirectory(Platform)
add_subdirectory(Project)
add_subdirectory(Tool)


########################
### Project Settings ###
########################

add_custom_target(debug_taniho)
add_dependencies(debug_taniho debug_taniho_binary)

add_executable(debug_taniho_binary
    Project/taniho/taniho_linux.cpp
)

add_custom_target(test_taniho)
add_dependencies(test_taniho test_taniho_binary)

add_executable(test_taniho_binary
    Test/mpl/test_led.cpp
)

add_test(
    NAME example_test
    COMMAND test_taniho_binary
)


target_link_libraries(debug_taniho_binary
    Library
    Linux
    taniho
    GitHash
)

target_link_libraries(test_taniho_binary
    gtest_main
)


##################
### GoogleTest ###
##################

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/06f44bc951046150f1348598854b211afdcf37fc.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
