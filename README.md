# Scorbot Hardware Interface

This project is a continuation of 'Project Cali' a mobile manipulator designed by students and staff of California State University Los Angeles (CSULA), and the members of the student organization 'American Society of Mechanical Engineers (ASME). Link: https://github.com/ASME-ground-robot/robot_cali. This platform is intended for reasearch, educational purposes, as well as design competitions. If you would like to read more about the overal project feel free to click on the link provided. I took part in development starting in May 2022, where I acted as a research assistant for a graduate level research, where I worked on developing this controller needed to manipulate the 'Scorbot ER III' robotic arm which was used in development of this platform . I then led further development of 'Project Cali' as a 'Project Lead' from September 2022 - May 2023, where I've done further work in navigation.

![Rover_Urc](https://github.com/JovanLukic79/scorbot_control_2/assets/115774118/f26800ed-8741-468d-a514-e1811fa5fde4)


This repo describes one of my particular contribution of the overall platform which is to provide a control scheme for the 6-link robotic arm integrated onto the overall platform. ROS's 'MoveIt' package was used to provide the control system, and GUI needed for robotic arm manipulation. ROS version that was used is ROS melodic. With that said, this package provides the code needed to interface ros_control and controllers with any sort of embedded controls. Later I may provide an example of said emedded controls just to demonstrate how this package would communicate with a set of hardware controls to acheive joint articulation. For now, I will just provide this part of the system for anyone who needs it as well as to use as my personal portfolio. This package can also be configured to any general robotic arm, so if you happen to need to interface your hardware controls with ROS's control system, feel free to use this as a template. With that said, a description of this package is provided bellow.


![IMG_1666](https://github.com/JovanLukic79/scorbot_control_2/assets/115774118/0d4ebff5-e54c-4485-bf71-09ed5fdbdc6d)

## Description

This repo is made up of two packages. The first one is "scorbot_control_2". This package is where I defined my hardware interface for the robotic arm. In addition to that,"scorbot_control_2" consists of the robotic arm simulation pacakge named "gazebo.launch". The second package is "scorbot_moveit_2", was built using "moveit_setup_assistants". This contains "move_group.launch" the main node needed to communicate with the rest of ROS's control system. In addition to movegroup, this package would also provide moveit_rviz.launch file which woutld act as the main GUI needed to control your robotic arm. 

## Demonstration
![robotic_arm_demo](https://github.com/JovanLukic79/scorbot_control_2/assets/115774118/fde7b4bf-d4aa-48ad-9057-2868b8ec43db)



## Scorbot Hardware Interface (Controller.launch)
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch scorbot_control_2 controller.launch 
```
or if you're running on simulation
```
roslaunch scorbot_control_2 controller.launch is_sim:=true
```

This launch file launches scorbot_interface_node which is the node that interfaces with the ros_control system, and this node was defined in "scorbot_interface.cpp" file. A publisher ("/arduino/arm_actuate") was created in order to have this node publish data in ones embedded controllers.

It also contains "angle_converter_cpp" node, which was created in order to convert raidians to degrees. This is defined in the "angle_converter.cpp" file and this is used becasue ROS uses radians, and arduino (for example) uses degrees. 

This launch file also contains the control manager that launches all the controllers being used. In this case, the controller being used to control both the arm joints and the gripper is "position_controllers/JointTrajectoryController". which is provided by ros_controllers.

## Simulation (gazebo.launch)
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch scorbot_control_2 gazebo.launch
```
This file launches the simulation used to simulate robotic arm articulation. Gazebo is being used as the main simulation framework, and the name of the urdf used to define scorbot's simulation build can be found in "scorbot_fixed_to_ground.urdf.xacro"

This file also lauches "arm_control_gui". A package that provides a gui in order to articulate each joint individually.

![robot_arm_sim](https://github.com/JovanLukic79/scorbot_control_2/assets/115774118/3892b64b-a1bb-4a35-8788-b7521585229c)


## RVIZ GUI (moveit_rviz.launch)
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch scorbot_moveit_2 moveit_rviz.launch
```
Contains the main RVIZ GUI used to diplay robotic arm controls, joint topics, ect

If the robotic arm does not show up during the initial lauch:
1) top left corner: File > open config > catkin_ws > src > scorbot_control_2 > scorbot_moveit_2 > launch > moveit.rviz

![robot_arm_gui](https://github.com/JovanLukic79/scorbot_control_2/assets/115774118/eb73f351-7609-4f60-84fd-fcbaad04d8b4)


## movegroup (move_group.launch)
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch scorbot_moveit_2 move_group.launch
```
part of the "scorbot_moveit_2" moveit package. This launch file launches the main move_group node which provides all the necessary topics needed to communicate with the rest of the control system.

Additions were made in order to have move_group work accordingly with scorbot_interface node.
## Getting Started

### Dependencies

* Unbuntu 18.04, ROS Melodic (however should work with ROS Noetic)
* ros_controllers: https://github.com/ros-controls/ros_controllers
* Moveit: https://moveit.ros.org/install/
* Ros Gazebo Pakcages: https://github.com/ros-simulation/gazebo_ros_pkgs

### Installing
```
cd catkin_ws/src
```
```
git clone https://github.com/JovanLukic79/scorbot_control_2
```
```
cd ..
```
```
catkin_make
```
### Executing program
Step 1: Launch controllers
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch scorbot_control_2 controller.launch is_sim:=true
```
Step 2: Launch move group
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch scorbot_moveit_2 move_group.launch
```
Step 3: Launch Simulation
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch scorbot_control_2 gazebo.launch
```
Step 3: Launch RVIZ GUI
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch scorbot_moveit_2 moveit_rviz.launch
```
top left corner: File > open config > catkin_ws > src > scorbot_control_2 > scorbot_moveit_2 > launch > moveit.rviz
## Help
Feel free to contact me: jovanlukic792@gmail.com

## Authors
Contributers: jovanlukic792@gmail.com (me)

## License

## Acknowledgments
