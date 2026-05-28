cmake_minimum_required(VERSION 3.8)
project(chess_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Square.msg
  msg/ChessMove.msg
  msg/BoardState.msg
  msg/DetectedPiece.msg
  srv/ParseVoiceCommand.srv
  srv/SetGripper.srv
  srv/AttachPiece.srv
  action/ExecuteChessMove.action
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_package()
