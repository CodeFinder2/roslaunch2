#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 15/02/2017

# Import all submodules typically used in launch modules:
from group import *
from launch import *
from node import *
from parameter import *
from machine import *
from environment import *
from test import *
from package import *
from logging import *


def main():
    """
    Defines the core logic (= Python based dynamic launch files) of roslaunch2. It does NOT create any Launch modules
    or the like.
    TODOs:
    - unique interface for root launch modules and child py files? -> should be not diff;
      how to treat command line args and "parameters" in a similar manner?
    :return: None
    """
    parser = LaunchParameter(description='roslaunch2 - Python based launch files for ROS')
    parser.add_argument('-p', '--package', default=str(), help='ROS package name to search for <launchfile>')
    parser.add_argument('--no-colors', default=False, action="store_true",
                        help='Do not use colored output during processing')
    parser.add_argument('-d', '--dry-run', default=False, action="store_true",
                        help='Just print the launch file to stdout, do not run roslaunch')
    parser.add_argument('launchfile', help='Python based launch file')
    args = parser.get_args()
    init_logger(not args.no_colors)


    # Add default extension:
    if not os.path.splitext(args.launchfile)[1]:
        args.launchfile += '.pyl'

    # ROS package name given? Try to resolve path to package and update <launchfile> path appropriately:
    if args.package:
        args.launchfile = Package(args.package).find(args.launchfile)

    # Import the launch module, generate the content, write it to a default launch file and invoke 'roslaunch':
    m = Package.import_launch_module(args.launchfile)
    content = m.main().generate()
    if not args.dry_run:
        import tempfile
        import roslaunch  # dry-run even works w/o ROS
        ftmp = tempfile.NamedTemporaryFile(mode='w', suffix='.launch', delete=False)
        launch_path = ftmp.name
        ftmp.write(content)
        ftmp.close()  # ftmp still exists *after* this line
        sys.argv.insert(1, launch_path)
        roslaunch.main(sys.argv)
        silent_remove(launch_path)
    else:
        print(content)
