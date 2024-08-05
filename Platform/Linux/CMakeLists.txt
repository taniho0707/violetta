cmake_minimum_required(VERSION 3.10)
add_library(PLATFORM STATIC
  observer.cpp
  tcp_client.cpp
  arm_math_linux.cpp
)
