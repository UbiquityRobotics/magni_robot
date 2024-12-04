cmake_minimum_required(VERSION 3.5)
project(magni_nav)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()