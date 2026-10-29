cmake_minimum_required(VERSION 3.8)
project(cm_station_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY robots/
  DESTINATION share/${PROJECT_NAME}/robots/
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf/
)

ament_package()
