cmake_minimum_required(VERSION 2.8)
project(map_generator)
enable_testing()

if(CMAKE_COMPILER_IS_GNUCXX)
  set(COMPILER_IS_GCC_COMPATIBLE ON)
elseif("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
  set(COMPILER_IS_GCC_COMPATIBLE ON)
endif()

if(COMPILER_IS_GCC_COMPATIBLE)
  include(CheckCXXCompilerFlag)
  check_cxx_compiler_flag("-std=c++11" SUPPORTS_CXX11)
  if(SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  else()
    message(FATAL_ERROR "Compiler doesn't support -std=c++11.")
  endif()
endif()

add_library(map_generator
  map_generator.cpp)

add_executable(
  map_generator_bin
  main.cpp)
set_target_properties(map_generator_bin PROPERTIES OUTPUT_NAME map_generator)

target_link_libraries(map_generator_bin map_generator)

add_executable(
  map_generator_test
  test.cpp)

target_link_libraries(map_generator_test map_generator)

add_subdirectory(examples)

add_test(map_generator_test map_generator_test)
