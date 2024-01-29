cmake_minimum_required(VERSION 3.0)
add_library(mpl STATIC
    mpl_led.cpp
    mpl_timer.cpp
    mpl_imu.cpp
    mpl_battery.cpp
)
