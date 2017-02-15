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
    import os.path
    """
    Defines the core logic (= Python based dynamic launch files) of roslaunch2. It does NOT create any
    launch modules or the like.
    :return: None
    """
    import os.path
    parser = argparse.ArgumentPer(description='roslaunch2 - Python based launch files for ROS')
    parser.add_argument('--no-colors', default=False, action="store_true",
                        help='Do not use colored output during processing')
    parser.add_argument('-d', '--dry-run', default=False, action="store_true",
                        help='Just print the launch file to stdout, do not run roslaunch')
    parser.add_argument('package', nargs='?', help='ROS package name to search for <launchfile>')
    parser.add_argument('launchfile', nargs='+', help='Python based launch file')
    parser.add_argument('--ros-args', default=False, action="store_true",
                        help='Display command-line arguments for this launch file')
    args, _ = parser.parse_known_args()
    init_logger(not args.no_colors)

    if len(args.launchfile) > 1:
        logging.warning("Multiple launch files at once are not supported (yet), just using the first.")

    args.launchfile = args.launchfile[0]

    # Add default extension:
    if not os.path.splitext(args.launchfile)[1]:
        args.launchfile += '.pyl'

    # ROS package name given? Try to resolve path to package and update <launchfile> path appropriately:
    if args.package:
        args.launchfile = Package(args.package).find(args.launchfile)

    # Import the launch module, generate the content, write it to a launch file and invoke 'roslaunch':
    m = Package.import_launch_module(args.launchfile)
    if args.ros_args:
        sys.argv[0] = args.launchfile  # display correct script name
        sys.argv.append('--usage')  # force launch module to output its help text
        m.main()
        return
    content = m.main().generate()
    if not args.dry_run:
        import tempfile
        import roslaunch  # dry-run even works w/o ROS
        ftmp = tempfile.NamedTemporaryFile(mode='w', suffix='.launch', delete=False)
        launch_path = ftmp.name
        ftmp.write(content)
        ftmp.close()  # close it so that roslaunch can open it (file still exists)
        sys.argv.insert(1, launch_path)
        # noinspection PyBroadException
        try:
            roslaunch.main(sys.argv)
        except:
            pass
        import utils
        utils.silent_remove(launch_path)
    else:
        print(content)
