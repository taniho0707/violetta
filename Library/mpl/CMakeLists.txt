cmake_minimum_required(VERSION 3.10)
add_library(mpl STATIC
    mpl_led.cpp
    mpl_timer.cpp
    mpl_imu.cpp
    mpl_battery.cpp
    mpl_wallsensor.cpp
    mpl_encoder.cpp
    mpl_debug.cpp
    mpl_motor.cpp
)
