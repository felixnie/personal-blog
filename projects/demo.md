---
layout: page
title: Demo
description: >
  Running some demos.
hide_description: false
sitemap: false
---

This chapter shows how to run **hector_quadrotor** and **tum_simulator** on ROS Kinetic and Gazebo 7.

## Running hector_quadrotor

The paper behind this package is **Comprehensive Simulation of Quadrotor UAVs using ROS and Gazebo**

To install on Kinetic, we can utilize the script [here](https://answers.ros.org/question/244776/is-it-possible-to-run-the-hector_quadrotor-demos-in-kinetic/):
~~~shell
#!/bin/bash

# to execute when in catkin_ws/src folder
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-unique-identifier
sudo apt-get install ros-kinetic-geographic-info
sudo apt-get install ros-kinetic-laser-geometry
sudo apt-get install ros-kinetic-tf-conversions
sudo apt-get install ros-kinetic-tf2-geometry-msgs
sudo apt-get install ros-kinetic-joy

git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor
git clone -b catkin https://github.com/tu-darmstadt-ros-pkg/hector_localization
git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_gazebo
git clone -b kinetic-devel https://github.com/tu-darmstadt-ros-pkg/hector_models
git clone -b catkin https://github.com/tu-darmstadt-ros-pkg/hector_slam

sed -i -e 's/option(USE_PROPULSION_PLUGIN "Use a model of the quadrotor propulsion system" ON)/option(USE_PROPULSION_PLUGIN "Use a model of the quadrotor propulsion system" OFF)/g' hector_quadrotor/hector_quadrotor/hector_quadrotor_gazebo/urdf/CMakeLists.txt

sed -i -e 's/option(USE_AERODYNAMICS_PLUGIN "Use a model of the quadrotor aerodynamics" ON)/option(USE_AERODYNAMICS_PLUGIN "Use a model of the quadrotor aerodynamics" OFF)/g' hector_quadrotor/hector_quadrotor/hector_quadrotor_gazebo/urdf/CMakeLists.txt

# this is to deactivate warnings
sed -i -e 's/add_dependencies(landing_action hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor/hector_quadrotor_actions/CMakeLists.txt

sed -i -e 's/add_dependencies(pose_action hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor/hector_quadrotor_actions/CMakeLists.txt

sed -i -e 's/add_dependencies(takeoff_action hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor/hector_quadrotor_actions/CMakeLists.txt

sed -i -e 's/add_dependencies(hector_quadrotor_controllers hector_uav_msgs_generate_message_cpp)//g' hector_quadrotor/hector_quadrotor/hector_quadrotor_controllers/CMakeLists.txt


cd ..
catkin_make
~~~

Launch the outdoor flight demo (remember to source):
~~~shell
cd catkin_ws/
source devel/setup.bash
roslaunch hector_quadrotor_demo outdoor_flight_gazebo.launch
~~~

Open a new terminal:
~~~shell
cd catkin_ws/
source devel/setup.bash
rosservice call \enable_motors true
~~~

Install teleop-twist-keyboard:
~~~shell
sudo apt-get install ros-kinetic-teleop-twist-keyboard
~~~

Run teleop-twist-keyboard:
~~~shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
~~~

## Running tum_simulator on Kinetic and Gazebo 7

[tum_simulator](http://wiki.ros.org/tum_simulator) is an implementation of a gazebo simulator for AR-Drone 2.0 developed by researchers from Computer Vision Group at the Technical University of Munich. It's better to read [gazebo tutorials](http://www.ros.org/wiki/simulator_gazebo/Tutorials/) before running this package. Instead of using joy_node and ardrone_joystick packages, we can use keyboard to operate.

Before started, we need to get the ROS driver for AR-Drone 1.0 & 2.0 quadrocopters:
~~~shell
apt-get install ros-kinetic-ardrone-autonomy
~~~

If you follow the guide, the installation by default is for the ROS Fuerte. But we can still directly follow the guide [here](http://wiki.ros.org/tum_simulator) since the author has tested this version on Ubuntu 16.04 and ROS Kinetic. Or we can instead use an updated version of tum_simulator, which can be found [here](https://github.com/eborghi10/AR.Drone-ROS).

Then we can use teleop_twist_keyboard to control like we did when running hector_quadrotor demo. Here are some issues you may encounter especially when you are using virtual machines for study. VM is not recommended for running simulations because no matter how powerful your graphic cards are, VM cannot use your graphic cards when your main OS is using them. This will make Gazebo running rate become extremely low.

### OpenML issue in VM

If you are using virtual machines like VMware, then OpenML issue may occur when running: 
~~~shell
roscore & rosrun gazebo_ros gazebo
~~~

~~~shell
“VMware: vmw_ioctl_command error Invalid argument.
Aborted (core dumped)
/opt/ros/indigo/lib/gazebo_ros/gazebo: 39: kill: invalid signal number or name: SIGINT”
~~~

One solution is to disable OpenGL support. This can be done by setting the environment variable SVGA_VGPU10=0.

~~~shell
export SVGA_VGPU10=0
~~~

Then we can happily run gazebo，roscore & rosrun gazebo_ros gazebo.

To make the change permanent, type:
~~~shell
echo "export SVGA_VGPU10=0" >> ~/.bashrc
~~~

### Gazebo Version Issue in VM

If you are using ROS Kinetic 7.0.0 in VMware Workstation, and Gazebo dies when trying to get image_view, then it's not your problem. This issue is solved in gazebo 7.4.0.

If we just upgrade Gazebo, the new version will be 7.15.0 and the problem will be gone.

To install 7.15.0, I just followed the guide [here](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=7.0).

The commands I used in ROS Kinetic and Ubuntu 16.04 are:

~~~shell
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7 -y
~~~

Continue with [Basics](basics.md){:.heading.flip-title}
{:.read-more}
