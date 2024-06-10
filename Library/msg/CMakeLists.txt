cmake_minimum_required(VERSION 3.10)
add_library(msg OBJECT
    msg_server.cpp
    msg_format.cpp
    msg_format_imu.cpp
    msg_format_encoder.cpp
    msg_format_wallsensor.cpp
    msg_format_battery.cpp
    msg_format_motor.cpp
    msg_format_motor_current.cpp
    msg_format_localizer.cpp
)
