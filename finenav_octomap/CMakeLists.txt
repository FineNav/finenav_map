cmake_minimum_required(VERSION 3.22)
project(finenav_octomap)

find_package(ament_cmake REQUIRED)
find_package(octomap REQUIRED)
find_package(finenav_common REQUIRED)

# Header-only wrapper around octomap::OcTree.
add_library(finenav_octomap INTERFACE)
target_compile_features(finenav_octomap INTERFACE cxx_std_20)
target_include_directories(finenav_octomap INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(finenav_octomap INTERFACE finenav_common)
target_link_libraries(finenav_octomap INTERFACE ${OCTOMAP_LIBRARIES})

install(TARGETS finenav_octomap EXPORT finenav_octomapTargets)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_octomap test/test_octomap.cpp)
  target_include_directories(test_octomap PRIVATE include)
  target_link_libraries(test_octomap finenav_octomap)
  ament_target_dependencies(test_octomap finenav_common)
endif()

ament_export_targets(finenav_octomapTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(finenav_common octomap)
ament_package()
