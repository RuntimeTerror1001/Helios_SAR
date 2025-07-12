cmake_minimum_required(VERSION 3.8)
project(helios_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ---------- library target ----------
add_library(helios_controller SHARED
  src/HeliosController.cpp
  src/helios_attitude_controller.cpp
  src/helios_pid.cpp
  src/helios_position_controller.cpp
  src/helios_utils.cpp
  src/pid_main.cpp)

# C sources need extern "C" when included from C++; tell CMake
set_target_properties(helios_controller PROPERTIES
  LINKER_LANGUAGE CXX)

target_include_directories(helios_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/include/helios_control
)

ament_target_dependencies(helios_controller
  ignition-gazebo6
  ignition-plugin1
  rclcpp
  geometry_msgs)

# ---------- installation ----------
install(TARGETS helios_controller
        LIBRARY DESTINATION lib)

install(DIRECTORY include/
        DESTINATION include)

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

ament_package()
