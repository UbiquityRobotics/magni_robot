cmake_minimum_required(VERSION 2.8.3)
project(magni_viz)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
