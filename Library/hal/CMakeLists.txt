cmake_minimum_required(VERSION 3.10)
add_library(hal STATIC
    hal_led.cpp
    hal_timer.cpp
    hal_imu.cpp
    hal_battery.cpp
    hal_wallsensor.cpp
    hal_encoder.cpp
    hal_debug.cpp
    hal_motor.cpp
)
