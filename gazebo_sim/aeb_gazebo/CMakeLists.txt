cmake_minimum_required(VERSION 3.5)
project(aeb_gazebo)

# ── Dependencies ────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── CAN message types ────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AebRadarTarget.msg"
  "msg/AebEgoVehicle.msg"
  "msg/AebBrakeCmd.msg"
  "msg/AebFsmState.msg"
  "msg/AebAlert.msg"
  DEPENDENCIES std_msgs
)

# ── Path to the embedded C sources ──────────────────────────────────────────
# CMAKE_CURRENT_SOURCE_DIR follows the symlink (~/aeb_ws/src/aeb_gazebo),
# so relative paths don't reach the Windows tree. Use the absolute WSL path.
set(AEB_C_ROOT "/mnt/c/Users/rfagu/Codigos/Pos/AEB/modeling/c_embedded"
    CACHE PATH "Path to the AEB embedded C source tree")

set(AEB_C_SOURCES
  ${AEB_C_ROOT}/src/aeb_main.c
  ${AEB_C_ROOT}/src/aeb_perception.c
  ${AEB_C_ROOT}/src/aeb_ttc.c
  ${AEB_C_ROOT}/src/aeb_fsm.c
  ${AEB_C_ROOT}/src/aeb_pid.c
  ${AEB_C_ROOT}/src/aeb_alert.c
)

# ── C++ controller node ──────────────────────────────────────────────────────
add_executable(aeb_controller_node
  src/aeb_controller_node.cpp
  ${AEB_C_SOURCES}
)

target_include_directories(aeb_controller_node PRIVATE
  ${AEB_C_ROOT}/include
)

ament_target_dependencies(aeb_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
)

# Link the generated message interfaces to the C++ node
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(aeb_controller_node
  "${cpp_typesupport_target}"
  m  # libm for math.h used by aeb_fsm.c / aeb_pid.c
)

install(TARGETS aeb_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python nodes ─────────────────────────────────────────────────────────────
install(PROGRAMS
  src/scenario_controller.py
  src/target_vehicle_controller.py
  src/dashboard_node.py
  src/dashboard_overlay.py
  src/perception_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Resource files ────────────────────────────────────────────────────────────
install(DIRECTORY
  launch
  worlds
  models
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
