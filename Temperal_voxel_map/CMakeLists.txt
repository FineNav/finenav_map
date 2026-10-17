cmake_minimum_required(VERSION 3.22)
project(Temperal_voxel_map)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(finenav_core REQUIRED)
find_package(finenav_grid_map REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

generate_parameter_library(Temperal_voxel_map_params config/Temperal_voxel_map.yaml)

add_library(Temperal_voxel_map SHARED
  src/Temperal_voxel_map.cpp
  src/plugin_registration.cpp
)
target_compile_features(Temperal_voxel_map PUBLIC cxx_std_20)
target_include_directories(Temperal_voxel_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(Temperal_voxel_map PUBLIC finenav_core finenav_grid_map rclcpp sensor_msgs)
target_link_libraries(Temperal_voxel_map PUBLIC Eigen3::Eigen Temperal_voxel_map_params)

install(TARGETS Temperal_voxel_map
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_Temperal_voxel_map test/test_Temperal_voxel_map.cpp)
  target_include_directories(test_Temperal_voxel_map PRIVATE include)
  target_link_libraries(test_Temperal_voxel_map Temperal_voxel_map)
  ament_target_dependencies(test_Temperal_voxel_map finenav_core rclcpp sensor_msgs)

  ament_add_gtest(test_plugin_registration test/test_plugin_registration.cpp)
  target_compile_definitions(test_plugin_registration PRIVATE
    TEMPERAL_VOXEL_MAP_LIBRARY="$<TARGET_FILE:Temperal_voxel_map>")
  target_link_libraries(test_plugin_registration ${CMAKE_DL_LIBS})
  ament_target_dependencies(test_plugin_registration finenav_core rclcpp)
endif()

ament_export_include_directories(include)
ament_export_libraries(Temperal_voxel_map)
ament_export_dependencies(eigen3_cmake_module finenav_core finenav_grid_map rclcpp sensor_msgs)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
