cmake_minimum_required(VERSION 3.10)
project(eigen_cal)

set(CMAKE_CXX_STANDARD 11)
include_directories("/usr/include/eigen3")
add_executable(eigen_cal main.cpp)