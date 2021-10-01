cmake_minimum_required(VERSION 3.0.2)
project(open_manipulator_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  sensor_msgs
  controller_manager
  hardware_interface
  joint_limits_interface
  transmission_interface
)

include_directories(
        include
        include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  roscpp std_msgs std_srvs 
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
         src/${PROJECT_NAME}/open_manipulator_control.cpp        
 )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)


add_executable(open_manipulator_control_node src/${PROJECT_NAME}/open_manipulator_control_node.cpp)
target_link_libraries(open_manipulator_control_node  ${PROJECT_NAME})
add_dependencies(open_manipulator_control_node ${PROJECT_NAME} )