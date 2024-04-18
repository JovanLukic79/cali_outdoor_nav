## 'Cali' Outdoor Navigation

This project is a continuation of 'Project Cali' a mobile manipulator designed by students and staff of California State University Los Angeles (CSULA), and the members of the student organization 'American Society of Mechanical Engineers (ASME). Link: https://github.com/ASME-ground-robot/robot_cali. This platform is intended for reasearch, educational purposes, as well as design competitions. If you would like to read more about the overal project feel free to click on the link provided. I took part in development starting in May 2022, where I acted as a research assistant for a graduate level research, where I worked on developing this controller needed to manipulate the 'Scorbot ER III' robotic arm which was used in development of this platform . I then led further development of 'Project Cali' as a 'Project Lead' from September 2022 - May 2023, where I've done further work in navigation. 

## Description

This repo describes one of my particular contribution of the navigation system intended for outdoor navigation. It uses a GPS odometry in order to perform localizastion. The localizzation algorithim that was used was "ekf_localization", from the "robot_localization" package. A 'sensor fusion' based localization algorithim that is able to perform sensor fusion based localizastion using either IMU, Wheel, and GPS odometry to generate 2D-State estimation [2]. In this build, I configured our localization to use GPS, and IMU. "Navsat_transform" algorithim was also used in order to perform the actual sensor fusion, and to turn the GPS/IMU data into "/odometry/fix" messages in order for "ekf_localization" to subscribe too. I added a simulated world 'cpr_orchard_gazebo' provided by 'Clear Path Robotics', in order to simulate 'Cali' robot in an outdoor agricultrual environment. Lastly, sensor models provided by 'Hector_gazebo_pluggins" were provided to simulate GPS,IMU sensor data [5].

I've also created a node that is able to send a waypoint to some specific location using cartesian coordinates. This doesn't serve much purpose but I decided to add it anyways. I also inteded to create my own coverage planner. Since i'm simulating 'Cali' rover in an 'Orchard field', I'd think it would've been fitting for 'Cali' to do some form of sweep/coverage planning. However, I just wanted to have this up so I could finally start doing projects in ROS2. With that said, the extent of this project is just to have 'robot_localization' configured to 'Project Cali', and to have it perform navigation (using '2D Nav goals') on simulation (Considering by the time I had it figured out, I already graduated and don't have access to the phyiscal rover).  

<ins>Dependencies (Install):</ins>
- 'Project Cali' Repo: https://github.com/ASME-ground-robot/robot_cali
- 'Robot_Localization': https://github.com/cra-ros-pkg/robot_localization (git clone) or "sudo apt-get insatll ros-melodic-robot-localization"
- Clear path Robotics 'cpr_orchard_gazebo" simulation world: https://github.com/clearpathrobotics/cpr_gazebo
  
## Demonstration
<ins>Desclaimer</ins>
I needed to lower the fps, and cut the recorded demonstrations to 30 seconds in order to fit this GIF into this repo.

![Navigation_demo(3)](https://github.com/JovanLukic79/cali_outdoor_nav/assets/115774118/fddcbc5d-a26d-4074-b853-151a522ddd3c)


## Cali Rover Simulation (gazebo.launch)
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch gps_waypoint gazebo.launch
```

This file laucnhes the gazebo simulation (of 'cali' in a middle of an orchard) along with the necessary GUI (that provides all the planner, map, odometry configuration).

![Gazwbo_ROver_2](https://github.com/JovanLukic79/cali_outdoor_nav/assets/115774118/1cde783d-0c2e-490d-8665-20dba575ec1a)


## Navigation (ekf_navigation.launch)
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch gps_waypoint ekf_navigation.launch
```
This file launches:
- move_group.launch : A launch file that launches 'move_group' node which allows us to use ROS's navigation system, and move_group messages
- navsat_transform_sim_2.launch: This file launches 'navsat_tansform' node
- ekf_loclaziation: This file launches 'ekf_localization' node  


## Bringup (launch everything at once)
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch gps_waypoint Map_start_2_test.launch
```
This file laucnhes both "gazebo.launch" and "ekf_navigation.launch" to launch everything all at once

## Getting Started

### Dependencies

* Unbuntu 18.04, ROS Melodic (however should work with ROS Noetic)
* ros_controllers: https://github.com/ros-controls/ros_controllers
* Moveit: https://moveit.ros.org/install/
* Ros Gazebo Pakcages: https://github.com/ros-simulation/gazebo_ros_pkgs
* 'Project Cali' Repo: https://github.com/ASME-ground-robot/robot_cali
* 'Robot_Localization': https://github.com/cra-ros-pkg/robot_localization (git clone) or "sudo apt-get insatll ros-melodic-robot-localization"
*  Clear path Robotics 'cpr_orchard_gazebo" simulation world: https://github.com/clearpathrobotics/cpr_gazebo

### Installing
```
cd catkin_ws/src
```
```
git clone https://github.com/JovanLukic79/scorbot_control_2](https://github.com/JovanLukic79/cali_outdoor_nav
```
```
cd ..
```
```
catkin_make
```
### Executing program
Step 1: Launch Simulation
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch gps_waypoint gazebo.launch
```
Step 2: Launch navigation 
```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch gps_waypoint ekf_navigation.launch

Step 3: Launch everything at once

```
cd catkin_ws
```
```
source devel/setup.bash
```
```
roslaunch gps_waypoint Map_start_2_test.launch
```

## Help
Feel free to contact me: jovanlukic792@gmail.com

## Authors
Contributers: jovanlukic792@gmail.com (me)

## License

## Acknowledgments
[1] https://github.com/ASME-ground-robot/robot_cali
[2] https://docs.ros.org/en/lunar/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf
[3] http://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
[4] https://github.com/clearpathrobotics/cpr_gazebo
[5] http://wiki.ros.org/hector_gazebo_plugins
