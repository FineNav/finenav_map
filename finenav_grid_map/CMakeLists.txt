cmake_minimum_required(VERSION 3.22)
project(finenav_grid_map)

find_package(ament_cmake REQUIRED)
find_package(finenav_common REQUIRED)

# Header-only template container.
add_library(finenav_grid_map INTERFACE)
target_compile_features(finenav_grid_map INTERFACE cxx_std_20)
target_include_directories(finenav_grid_map INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(finenav_grid_map INTERFACE finenav_common)

install(TARGETS finenav_grid_map EXPORT finenav_grid_mapTargets)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_grid_map test/test_grid_map.cpp)
  target_include_directories(test_grid_map PRIVATE include)
  target_link_libraries(test_grid_map finenav_grid_map)
  ament_target_dependencies(test_grid_map finenav_common)
endif()

ament_export_targets(finenav_grid_mapTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(finenav_common)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
