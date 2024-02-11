cmake_minimum_required(VERSION 3.0)

add_library(STM32F411xE STATIC
    stm32f4xx_it.cpp
    system_stm32f4xx.c
)
