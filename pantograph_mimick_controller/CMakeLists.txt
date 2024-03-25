cmake_minimum_required(VERSION 3.8)
project(pantograph_mimick_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

set(DEPS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  pantograph_library
)

foreach(Dependency IN ITEMS ${DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  pantograph_mimick_controller_parameters
  src/pantograph_mimick_controller_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/pantograph_mimick_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  pantograph_mimick_controller_parameters
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "PANTOGRAPH_MIMICK_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface pantograph_mimick_controller_plugin.xml)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
  pantograph_mimick_controller
  pantograph_mimick_controller_parameters
  EXPORT export_pantograph_mimick_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_pantograph_mimick_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})
ament_package()
