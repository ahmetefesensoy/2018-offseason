cmake_minimum_required(VERSION 3.8)
project(frc_autonomy_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AutonomyCommand.msg"
  "msg/AutonomyStatus.msg"
  "msg/AlliancePlan.msg"
  "msg/GameObject.msg"
  "msg/PlanSegment.msg"
  "msg/PredictedPose.msg"
  "msg/ReservationSample.msg"
  "msg/ReservationTube.msg"
  "msg/ReservationTubeArray.msg"
  "msg/RobotTrack.msg"
  "msg/RobotPlan.msg"
  "msg/SemanticObservation.msg"
  "msg/SemanticObservationArray.msg"
  "msg/TimedPose.msg"
  "msg/WorldState.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
