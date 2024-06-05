## Object-Tracking-with-Robotic-Navigation
This repository contains the implementation of a robotic system designed to follow a person using two control algorithms: Proportional Control (P-Control) and Proportional-Derivative Control (PD-Control). The system leverages YOLO (You Only Look Once) for real-time object detection to identify and track the target.

### Key Components

1. **YOLO for Object Detection**:
   - Utilizes the YOLO object detection model to detect a target in the robot's camera feed. 

2. **Proportional Control (P-Control)**:
   - A simple control algorithm that adjusts the robot's movement based on the proportional error between the detected target's position and the desired position.
     
3. **Proportional-Derivative Control (PD-Control)**:
   - An advanced control algorithm that improves upon P-Control by adding a derivative term. This term accounts for the rate of change of the error, allowing for smoother and more responsive tracking, reducing oscillations and improving stability.

### System Architecture

The system is composed of several ROS nodes:

- **Camera Feed Node**:
  - Captures the video stream from the robot's camera and processes it using YOLO to detect the target.

- **Follower Node**:
  - Implements the P-Control and PD-Control algorithms to adjust the robot's velocity based on the detected target's position.

- **Object Controller Node**:
  - Used to move the target around.


## Installing Prerequisites
### Install ROS (Noetic)
```
$ sudo apt update
$ sudo apt upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_noetic.sh
$ chmod 755 ./install_ros_noetic.sh 
$ bash ./install_ros_noetic.sh
```

### Install Dependent Packages
```
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

### Install Turtlebot3 and Gazebo Packages
```
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
$ sudo apt install ros-noetic-gazebo-ros-pkgs 
```

## Creating and Building the package
Create a catkin workspace and a folder named `src` inside it. 
```
cd ~/catkin_ws/src/
$ git clone https://github.com/arushikhokhar/Object-Tracking-with-Robotic-Navigation
$ cd ~/catkin_ws && catkin_make
$ source devel/setup.bash
```

## Launching the Simulation
First, launch the world. This world has a person's model in it by default. If you use another model instead of a person, you need to modify the detection logic in camera_feed.py accordingly. Refer to [Line 46](https://github.com/arushikhokhar/Object-Tracking-with-Robotic-Navigation/blob/main/turtlebot3_simulations/turtlebot3_gazebo/src/camera_feed.py) and change the name based on how it is detected by YOLO.
`roslaunch turtlebot3_gazebo person_world.launch`

Now, either use `roslaunch turtlebot3_gazebo p_control.launch` for object following using the proportional controller or use `roslaunch turtlebot3_gazebo pd_control.launch` for object following using the proportional derivative controller.

## Moving the target object
You'll now be able to move the target object around using yourr keyboard. Use the 'W', 'S', 'A', 'D' keys for forward, backward, left and right movements. Use the 'Q' and 'E' keys for clockwise and anti-clockwise rotations. Note, the controls might act a bit counter-intuititve due to different orientation of every model when the simulation begins.

## Output
### Proportional Controller
![Proportional Controller](https://github.com/arushikhokhar/Object-Tracking-with-Robotic-Navigation/blob/main/assets/p-control.gif)
### Proportional Derivative (PD) Controller
![Proportional Derivative Controller](https://github.com/arushikhokhar/Object-Tracking-with-Robotic-Navigation/blob/main/assets/pd-control.gif)



