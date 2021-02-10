---
layout: page
title: Basics
description: >
  Basics of ROS.
hide_description: false
sitemap: false
---

This chapter covers the most frequently used ROS commands.

## ROS File System
rospack allows you to get information about packages. In this tutorial, we are only going to cover the find option, which returns the path to package.
> rospack find [package_name]

It allows you to change directory (cd) directly to a package or a stack
> roscd [locationname[/subdir]]

It allows you to ls directly in a package by name rather than by absolute path.
$ rosls [locationname[/subdir]]

## Creating a workspace for catkin
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

## Creating a ROS Package
The simplest possible package might have a structure which looks like this:
```
my_package/
  CMakeLists.txt
  package.xml
```
The recommended method of working with catkin packages is using a catkin workspace, but you can also build catkin packages standalone. A trivial workspace might look like this:
```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

Continue with [Configuration](config.md){:.heading.flip-title}
{:.read-more}
