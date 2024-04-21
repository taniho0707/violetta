cmake_minimum_required(VERSION 3.10)

add_library(PLATFORM OBJECT
# add_library(STM32F411xE STATIC
    stm32f4xx_it.cpp
    system_stm32f4xx.c
    syscalls.c
    sysmem.c
)
