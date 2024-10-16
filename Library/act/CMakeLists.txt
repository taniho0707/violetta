cmake_minimum_required(VERSION 3.10)

if(${MOUSE} STREQUAL LazuliSensor)
  add_library(act STATIC
    act_manager.cpp
    act_none.cpp
    act_wallsensor_run.cpp
  )
else()
  add_library(act STATIC
    act_initialize.cpp
    act_debug.cpp
    act_manager.cpp
    act_module_test.cpp
    act_none.cpp
    act_paramtune_motor.cpp
    act_search.cpp
    act_select_next.cpp
    act_shortrun.cpp
    act_wallsensor_check.cpp
    act_wallsensor_run.cpp
    act_enkaigei.cpp
    act_standby.cpp
    act_system_identification.cpp
  )
endif()
