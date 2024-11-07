cmake_minimum_required(VERSION 3.10)

if(${PLATFORM} STREQUAL Linux)
    add_subdirectory(Linux)
else()
    if(${MOUSE} STREQUAL LazuliSensor)
        add_library(hal STATIC
            hal_timer.cpp
            hal_wallsensor.cpp
        )
    else()
        add_library(hal STATIC
            hal_led.cpp
            hal_timer.cpp
            hal_imu.cpp
            hal_battery.cpp
            hal_wallsensor.cpp
            hal_encoder.cpp
            hal_debug.cpp
            hal_motor.cpp
            hal_speaker.cpp
        )
    endif()
endif()