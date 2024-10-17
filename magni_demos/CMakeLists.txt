cmake_minimum_required(VERSION 3.5)
project(magni_demos)

find_package(ament_cmake REQUIRED)

#############
## Install ##
#############

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
