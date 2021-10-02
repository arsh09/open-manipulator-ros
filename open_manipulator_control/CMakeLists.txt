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
  open_manipulator_6dof_libs
  robotis_manipulator

)
find_package(Boost REQUIRED)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp std_msgs open_manipulator_msgs robotis_manipulator open_manipulator_6dof_libs cmake_modules
        DEPENDS Boost
)


################################################################################
# Build
################################################################################
include_directories(
        include
        include/${PROJECT_NAME}
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}_hw_inteface
         src/open_manipulator_hw_interface.cpp        
 )

target_link_libraries(${PROJECT_NAME}_hw_inteface
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
)

## Declear a C++ node
add_executable(open_manipulator_control_node src/open_manipulator_control_node.cpp)
target_link_libraries(open_manipulator_control_node  ${PROJECT_NAME}_hw_inteface)
add_dependencies(open_manipulator_control_node ${PROJECT_NAME}_hw_inteface )