cmake_minimum_required(VERSION 3.10)
add_library(PLATFORM OBJECT
  observer.cpp
  # tcp_client.cpp
  udp_client.cpp
  arm_math_linux.cpp
)
