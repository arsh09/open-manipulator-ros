### Introduction

A rather simpler version for Dynamixel actuator based robot's ROS package. The robotis manipulator package is basically wrapped around ros-control hardware interface class so that trajectory controllers can be used (for both simulation and real) 

### Uuse

After normal catkin_make stuff: 

```bash
$ roslaunch open_manipulator_bringup open_manipulator_bringup.launch sim:=true
```

Currently simulation works with gripper and arm. 

For mimic joint in simulation, roboticsg group's gazebo plugins are used. 

