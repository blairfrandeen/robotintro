cmake_minimum_required(VERSION 3.22)
project(robotintro)
add_executable(robotintro main.cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

include_directories(../eigen)

