# FSR Technical project
This is the technical project for the Field and Service Robotics exam, at
University Federico II of Naples.

## Introduction
The aim of this project is to develop a system able to perform motion planning
and control for a **VTOL UAV**. In particular, an *AscTec Hummingbird*
quadcopter has been considered; however, the system can work with any
other UAV.

Motion planning is performed via **artificial potential** method, with a custom
**virtual point**-based algorithm to avoid local minima.
The control is achieved through a **passivity-based hierarchical controller**.
The estimation of extrernal wrench is achieved through a **momentum-based** first-order estimator.

## Installation Instructions - Ubuntu 18.04 with ROS Melodic
---------------------------------------------------------
 1. Install and initialize ROS melodic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-melodic-desktop-full ros-melodic-joy ros-melodic-octomap ros-melodic-octomap-msgs ros-melodic-octomap-ros ros-melodic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-melodic-control-toolbox ros-melodic-mavros ros-melodic-mav-msgs ros-melodic-map-server
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/melodic/setup.bash
 ```

**Please note**: This project has been developed and tested on Ubuntu 18.04 LTS, with ROS Melodic and Gazebo 9.0. It *could* work also on the newest versions of the softwares, but this has not been tested and thus is not guaranteed.

 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 ```

  > **Note** On OS X you need to install yaml-cpp using Homebrew `brew install yaml-cpp`.

 3. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin build
   ```

 4. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

**Please note**: it is possible that the package will not compile on the first
attempt. This is due to a weird behaviour of catkin_make, that compiles custom
msgs *after* the nodes. To solve this problem, open
[quad_control/CMakeLists.txt](quad_control/CMakeLists.txt), comment out the
lines (164 - 178), and compile: this will only compile the messages. Then,
uncomment those lines and compile again. This issue could be solved using
another package that only contains the messages.

## Play the scene
In order to play the provided scene, use the main launch file:
```
    roslaunch quad_control start.launch
```

If order to scope uav signals, set **scope** as true
```
    roslaunch quad_control start.launch scope:=true
```

## In detail documentation
In the doc folder you can find [a detailed report](quad_control/doc/Report.pdf)
on the project and [a video demonstration](quad_control/doc/video.mp4).