project(ukf_tracker)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/ukf_tracker.cpp
    src/main.cpp)

add_executable(ukf_tracker ${sources})
