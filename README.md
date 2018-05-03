# Overview
roslaunch2 is a (pure Python based) ROS package that facilitates writing versatile, flexible and dynamic launch configurations for the Robot Operating System (ROS 1) using Python, both for simulation and real hardware setups, as contrasted with the existing XML based launch file system of ROS, namely roslaunch. Note that roslaunch2 is not (yet) designed and developed for ROS 2 but for ROS 1 only although it may also inspire the development (of the launch system) of ROS 2. It is compatible with all ROS versions providing roslaunch which is used as its backend. roslaunch2 has been tested and heavily used on ROS Indigo, Jade, Kinetic, and Lunar; it also supports a “dry-mode” to generate launch files without ROS being installed at all. The key features of roslaunch2 are
- versatile control structures (conditionals, loops),
- extended support for launching and querying information remotely,
- an easy-to-use API for also launching from Python based ROS nodes dynamically, as well as
- basic load balancing capabilities for simulation setups.

**More information and detailed examples** can be found in the "roslaunch2: Versatile, Flexible and Dynamic Launch Configurations for the Robot Operating System" chapter of the "Robot Operating System (ROS): The Complete Reference" book (volume 4) published by Springer.
 
# Documentation & Tutorials
You may also want to have a look at the [documentation](https://codefinder2.github.io/roslaunch2/) and the [FAQ](https://github.com/CodeFinder2/roslaunch2/blob/master/doc/faq.md).

Setting up a *systemd* deamon (Ubuntu >= v15.10) for Pyro and the roslaunch2 server is described [here](https://github.com/CodeFinder2/roslaunch2/blob/master/config/systemd/README.md). [This guide](https://github.com/CodeFinder2/roslaunch2/blob/master/config/upstart/README.md) describes how to use *upstart* (Ubuntu < v15.10) to automatically run the Pyro name server and the roslaunch2 server on boot.

# License
The entire code is BSD 3-Clause licenced, see [here](https://github.com/CodeFinder2/roslaunch2/blob/master/LICENSE). Thus, you can modifiy it, use it privately and commercially but you must retain a license and copyright notice when doing so.

Copyright (c) 2018, Adrian Böckenkamp. All rights reserved.
