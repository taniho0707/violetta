cmake_minimum_required(VERSION 3.10)

add_library(PLATFORM OBJECT
    stm32l4xx_it.cpp
    system_stm32l4xx.c
    syscalls.c
    sysmem.c
)
