cmake_minimum_required(VERSION 3.8)
project(pantograph_library)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)



#-----------------------------------------------------
#   Install Python package
#-----------------------------------------------------
ament_python_install_package(
  ${PROJECT_NAME}
)

#-----------------------------------------------------
#   Export cpp library
#-----------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/pantograph_model.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC Eigen3)
target_compile_definitions(${PROJECT_NAME} PRIVATE "PANTOGRAPH_LIBRARY_BUILDING_LIBRARY")

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
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(Eigen3)
ament_package()

#-----------------------------------------------------
# Config for using GoogleTest
#-----------------------------------------------------

include(FetchContent)
fetchcontent_declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
fetchcontent_makeavailable(googletest)

enable_testing()

add_executable(
  test_panto_model
  tests/test_panto_model.cc
)
ament_target_dependencies(test_panto_model PUBLIC Eigen3)
target_link_libraries(
  test_panto_model
  GTest::gtest_main
  pantograph_library
)

include(GoogleTest)
gtest_discover_tests(test_panto_model)

# Add include folder
# Where Google Test's .h files can be found.
set(gtest_build_include_dirs
  "${PROJECT_SOURCE_DIR}/include"
  "${PROJECT_SOURCE_DIR}")
include_directories(${gtest_build_include_dirs})
