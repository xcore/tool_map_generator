cmake_minimum_required(VERSION 2.8)
project(map_generator)

add_library(map_generator
  map_generator.cpp)

add_executable(
  map_generator_bin
  main.cpp)

target_link_libraries(map_generator_bin map_generator)
