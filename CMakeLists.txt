cmake_minimum_required(VERSION 2.8.3)
project(rtt_joint_traj_generator_kdl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rtt_ros
  std_msgs
  kdl_parser
  rtt_ros_kdl_tools
  trajectory_msgs
  cmake_modules
)

find_package(orocos_kdl REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${USE_OROCOS_INCLUDE_DIRS}
  ${OROCOS_KDL_INCLUDE_DIRS}
)

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_link_libraries(${PROJECT_NAME} ${OROCOS_KDL_LIBRARIES} ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES})

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

orocos_generate_package(
  INCLUDE_DIRS include
  DEPENDS rtt_ros
)
