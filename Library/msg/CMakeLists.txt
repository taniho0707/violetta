cmake_minimum_required(VERSION 3.10)

if(${MOUSE} STREQUAL LazuliSensor)
    add_library(msg OBJECT
        msg_server.cpp
        msg_format.cpp
        msg_format_wallsensor.cpp
        msg_format_wall_analyser.cpp
    )
else()
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
        msg_format_motor_controller.cpp
        msg_format_wall_analyser.cpp
        msg_format_motor_controller_internal.cpp
    )
endif()
