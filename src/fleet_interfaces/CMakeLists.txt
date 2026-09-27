cmake_minimum_required(VERSION 3.8)
project(fleet_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/Task.msg"
  "msg/TaskState.msg"
  "msg/TaskPool.msg"
  "msg/RobotState.msg"
  "msg/MaxSumMessage.msg"
  "msg/TaskOwnership.msg"
  "msg/Bundle.msg"
  "msg/Intent.msg"
  "msg/PickupObservation.msg"
  "msg/Reservation.msg"
  "msg/P2PMessage.msg"
  "msg/P2PBeacon.msg"
  "msg/P2PEvent.msg"
  "msg/P2PNeighborList.msg"
  "msg/RobotWorldEntry.msg"
  "msg/TaskWorldEntry.msg"
  "msg/WorldView.msg"
)

set(srv_files
  "srv/CreateTask.srv"
  "srv/GenerateSampleTasks.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
