cmake_minimum_required(VERSION 3.5)
project(magni_viz)

find_package(ament_cmake REQUIRED)

#catkin_package()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()