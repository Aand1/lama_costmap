cmake_minimum_required(VERSION 2.8.4)
project(nj_costmap_tests)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(getindex getindex.cpp)
add_executable(coding coding.cpp)
