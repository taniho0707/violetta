cmake_minimum_required(VERSION 3.0)
add_library(hal STATIC
    hal_led.cpp
    hal_timer.cpp
    hal_imu.cpp
    hal_battery.cpp
    hal_wallsensor.cpp
    hal_encoder.cpp
)
