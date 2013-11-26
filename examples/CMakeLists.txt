include_directories(${CMAKE_CURRENT_SOURCE_DIR}/..)

add_executable(
  load_store_map
  load_store_map.cpp)

target_link_libraries(load_store_map map_generator)
