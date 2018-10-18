# Table of Contents  
* [Overview](#overview)
* [Installation](#install)
* [Example](#example)
* [Documentation & Tutorials](#docs)
* [License & Citing](#cite)

# Overview <a name="overview"/>
roslaunch2 is a (pure Python based) ROS package that facilitates writing **versatile, flexible and dynamic launch configurations for the Robot Operating System (ROS 1) using Python**, both for simulation and real hardware setups, as contrasted with the existing XML based launch file system of ROS, namely [roslaunch](http://wiki.ros.org/roslaunch). Note that roslaunch2 is not (yet) designed and developed for ROS 2 but for ROS 1 only although it may also inspire the development (of the launch system) of ROS 2. It is **compatible with all ROS versions providing roslaunch** which is used as its backend. roslaunch2 has been tested and heavily used on ROS Indigo, Jade, Kinetic, and Lunar; it also supports a “dry-mode” to generate launch files without ROS being installed at all. The **key features** of roslaunch2 are
- versatile control structures (conditionals, loops),
- extended support for launching and querying information remotely,
- an easy-to-use API for also launching from Python based ROS nodes dynamically, as well as
- basic load balancing capabilities for simulation setups.

**More information and detailed examples** can be found in the "roslaunch2: Versatile, Flexible and Dynamic Launch Configurations for the Robot Operating System" chapter of the "Robot Operating System (ROS): The Complete Reference" book (volume 4) published by Springer. [Please cite it (see below)](https://github.com/CodeFinder2/roslaunch2#license--citing), if you find roslaunch2 useful for your work.

# Installation <a name="install"/>
Installing the *roslaunch2* package is straightforward and equal to installing any other ROS package from source / GitHub.
1. Assuming, you already have a catkin workspace (see [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) how to setup one), change to that workspace's source directory: `cd ~/catkin_ws/src`
2. Now, clone this repository to that `src` directory: `git clone https://github.com/CodeFinder2/roslaunch2.git`
3. The package should be usable now. If not, run `catkin_make` once and ensure that the workspace is [sourced in your shell](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

For more information, see also:
- [Setting up a ROS package from Git](https://wiki.nps.edu/display/RC/Setting+up+a+ROS+package+from+Git)
- [Installing Packages - Download and Build a Package from Source](https://ros-industrial.github.io/industrial_training/_source/session1/Installing-Existing-Packages.html#download-and-build-a-package-from-source)

# Example <a name="example"/>
The following code shows a **small working example** that launches the ['fake_localization'](http://wiki.ros.org/fake_localization) node if installed (taken from the example of the aforementioned chapter).
```Python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from roslaunch2 import *

def main(**kwargs):  # contains the entire code to launch
    cfg = Launch()  # root object of the launch hierarchy
    # Process arguments (not command line arguments) for this launch module:
    ns = kwargs['namespace'] if 'namespace' in kwargs else str()

    g = Group(ns)  # possibly empty namespace group

    # Create a (cached) package reference:
    pkg = Package('fake_localization', True)
    if pkg and pkg.has_node('fake_localization', True):  # only add if it exists
        n = Node(pkg)
        # Set coodinate frame IDs (on the ROS parameter server):
        n += ServerParameter('global_frame_id', 'map')
        n += ServerParameter('odom_frame_id', tf_join(ns, 'odom'))
        n += ServerParameter('base_frame_id', tf_join(ns, 'base_link'))
        g += n  # move node to namespace 'ns'

    cfg += g
    return cfg
```
You can run it with:
```shell
$ roslaunch2 roslaunch2 rl2-example.pyl
```

# Documentation & Tutorials <a name="docs"/>
You may also want to have a look at the [**documentation**](https://codefinder2.github.io/roslaunch2/) and the [FAQ](https://github.com/CodeFinder2/roslaunch2/blob/master/doc/faq.md).

Setting up a [*systemd*](https://wiki.ubuntu.com/systemd) deamon (Ubuntu >= v15.10) for [Pyro](https://pythonhosted.org/Pyro4/) and the roslaunch2 server is described [here](https://github.com/CodeFinder2/roslaunch2/blob/master/config/systemd/README.md). [This guide](https://github.com/CodeFinder2/roslaunch2/blob/master/config/upstart/README.md) describes how to use [*upstart*](http://upstart.ubuntu.com/wiki/) (Ubuntu < v15.10) to automatically run the [Pyro name server](https://pythonhosted.org/Pyro4/nameserver.html) and the roslaunch2 server on boot.

To enable [tab completion support](https://en.wikipedia.org/wiki/Command-line_completion), you need to `source` the file `roslaunch2/config/roslaunch2_auto_completion.bash`.

# License & Citing <a name="cite"/>
The entire code is **BSD 3-Clause licenced**, see [here](https://github.com/CodeFinder2/roslaunch2/blob/master/LICENSE). Thus, you can modifiy it, use it privately and commercially but you must retain a license and copyright notice when doing so. If you find this package useful, **please cite** the aforementioned book chapter with (yet missing details will be added when  the publication process has been completed, stay tuned):
```
@INCOLLECTION{Boeckenkamp2019,
	author      = {B{\"o}ckenkamp, Adrian},
	title       = {{roslaunch2: Versatile, Flexible and Dynamic Launch Configurations for the Robot Operating System}},
	editor      = {Koubaa, Anis},
	booktitle   = {Robot Operating System (ROS): The Complete Reference},
	publisher   = {Springer International Publishing},
	address     = {Heidelberg},
	year        = 2019,
	volume      = 4,
	pages       = {TODO-TODO},
	chapter     = {TODO},
	isbn        = {TODO},
	doi         = {TODO},
	url         = {TODO}
}
```

Copyright (c) 2018, Adrian Böckenkamp, Department of Computer Science VII, TU Dortmund University.

All rights reserved.
