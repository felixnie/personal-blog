---
layout: page
title: Install
description: >
  Installing ROS and Gazebo with Qt Creator Plug-In
hide_description: false
sitemap: false
---

## Install ROS Kinetic Kame

Follow the instructions on ROS Wiki: http://wiki.ros.org/kinetic/Installation/Ubuntu
Please note that Ubuntu 16.04 is indispensable for this version of ROS. Although Gazebo is always paying attention on backwards compatibility, ROS itself is pretty troublesome when switching between versions.

## Install Gazebo

Usually Gazebo comes with a full ROS installation. We can directly run from terminal:

~~~shell
gazebo
~~~

If you missed your Gazebo or need a reinstallation, remember to select the compatible version of ROS Kinetic like:

~~~shell
sudo apt-get install gazebo7
~~~

## Install ROS Qt Creator Plug-in

They provide a GUI installation. Use Xenial Online Installer:

~~~shell
https://qtcreator-ros.datasys.swri.edu/downloads/installers/xenial/qtcreator-ros-xenial-latest-online-installer.run
~~~

We can then run a [Demo]{:.heading.flip-title}.

[demo]: demo.md