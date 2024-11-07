cmake_minimum_required(VERSION 3.10)

add_library(PLATFORM OBJECT
    stm32c0xx_it.cpp
    system_stm32c0xx.c
    syscalls.c
    sysmem.c
)
