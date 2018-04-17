#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslaunch2 as rl2
import rospy

if __name__ == '__main__':
    # Initialize the node:
    rospy.init_node('my_python_ros_node')
    # Do something in your node ...
    # Launch another node (fake_localization):
    lm = rl2.Package.include('roslaunch2', 'rl2-example', namespace='my_robot')
    rl2.start(lm)
