cmake_minimum_required(VERSION 3.8)
project(frc_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf rviz DESTINATION share/${PROJECT_NAME})

ament_package()
