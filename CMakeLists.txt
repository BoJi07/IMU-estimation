cmake_minimum_required(VERSION 3.5.1)

set(CMAKE_CXX_STANDARD 14)

project(IMU_est)

add_executable(IMU_est src/kalmanPos.cpp src/main.cpp)   
