---
layout: page
title: Quaternion
description: >
  Understanding quaternion and transform into understandable orientation.
hide_description: false
sitemap: false
---

ROS uses quaternions to track and apply rotations. A quaternion has 4 components (x,y,z,w). It's easy for humans to think of rotations about axes but hard to think in terms of quaternions. A suggestion is to calculate target rotations in terms of (roll about an X-axis) / (subsequent pitch about the Y-axis) / (subsequent yaw about the Z-axis), then convert to a quaternion:

~~~pyhton
# tf.transformations alternative is not yet available in tf2
from tf.transformations import quaternion_from_euler
  
if __name__ == '__main__':

  # RPY to convert: 90deg, 0, -90deg
  q = quaternion_from_euler(1.5707, 0, -1.5707)

  print "The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3])
~~~

Also there're functions in C++/Pyhton that converts quaternion back to Euler angles or directly into RPY (roll, pitch, yaw) like [this](https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac).

## Theory

Visualizing quaternions (4d numbers) with stereographic projection

https://www.youtube.com/watch?v=d4EgbgTm0Bg&ab_channel=3Blue1Brown

Why can't Euler angle do the work? There'll be a problem called [Gimbal lock](https://en.wikipedia.org/wiki/Gimbal_lock).

Gimbal locked airplane. When the pitch (green) and yaw (magenta) gimbals become aligned, changes to roll (blue) and yaw apply the same rotation to the airplane.

![Full-width image](https://upload.wikimedia.org/wikipedia/commons/4/49/Gimbal_Lock_Plane.gif){:.lead width="200px" height="200px" }