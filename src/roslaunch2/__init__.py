#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 13/03/2018

# Import all submodules typically used in launch modules:
from group import *
from parameter import *
from machine import *
from package import *
from logger import *
from utils import *
from remote import *
from launch import *
from node import *
from environment import *
from test import *

import argparse

__version__ = '0.1'

# Define the events in roslaunch2 that may be associated with custom actions:
on_initialize = Observable()
on_terminate = Observable()


def _strip_args(launch_path):
    """
    Return preprocessed argument list only containing options valid in roslaunch (not roslaunch2). Also append
    path to generated launch file. For example, this returns
    ['/home/user/catkin_ws/devel/bin/roslaunch2', '/tmp/tmpd8xFTj.launch', '--timeout=5']

    :param launch_path: path to generated launch file
    :return: list of options to be passed to roslaunch.main()
    """
    import sys
    dummy = argparse.ArgumentParser()
    # Create list of all options accepted by roslaunch (as of 15/02/17, ROS Kinetic). These information may also be
    # retrieved by "from roslaunch import _get_optparse" and then calling _get_optparse(), see
    # https://github.com/ros/ros_comm/blob/kinetic-devel/tools/roslaunch/src/roslaunch/__init__.py#L113
    # However, since this is not a stable API, we'll add them manually.
    dummy.add_argument('--files')
    dummy.add_argument('--args')
    dummy.add_argument('--find-node')
    dummy.add_argument('-c', '--child')
    dummy.add_argument('--local')
    dummy.add_argument('--screen')
    dummy.add_argument('-u', '--server_uri')
    dummy.add_argument('--run_id')
    dummy.add_argument('--wait')
    dummy.add_argument('-p', '--port')
    dummy.add_argument('--core')
    dummy.add_argument('--pid')
    dummy.add_argument('-v')
    dummy.add_argument('--dump-params')
    dummy.add_argument('--skip-log-check')
    dummy.add_argument('--disable-title')
    dummy.add_argument('-w', '--numworkers')
    dummy.add_argument('-t', '--timeout')
    _, unknown_args = dummy.parse_known_args()
    args = [arg for arg in sys.argv if arg not in unknown_args]
    args.insert(1, launch_path)
    return args


def _argument_parser(parents=None):
    """
    Defines the command line argument parser for roslaunch2.

    :param parents: Parents of argparse.ArgumentParser (not needed in most of the cases)
    :return: argparse.ArgumentParser instance
    """
    if parents is None:
        parents = []
    parser = argparse.ArgumentParser(description='roslaunch2 - Python based launch files for ROS (1)',
                                     add_help=False, parents=parents,
                                     conflict_handler='resolve' if parents else 'error')
    parser.add_argument('-h', '--help', default=False, action="help",
                        help='Show this help message and exit')
    parser.add_argument('--no-colors', default=False, action="store_true",
                        help='Do not use colored output during processing')
    parser.add_argument('--version', action='version', version='%(prog)s v{version}, \
                        (C) Copyright Adrian Böckenkamp, 16/02/2017'.format(version=__version__))
    parser.add_argument('-d', '--dry-run', default=False, action="store_true",
                        help='Just print the launch file to stdout, do not run roslaunch')
    parser.add_argument('package', nargs='?', help='ROS package name to search for <launchfile>')
    parser.add_argument('launchfile', nargs='+', help='Python based launch file')
    parser.add_argument('--ros-args', default=False, action="store_true",
                        help='Display command-line arguments for this launch file')
    return parser


def start(launch_obj, dry_run=False, silent=False):
    """
    Generates a temporary roslaunch XML file from the given roslaunch2 Launch instance and passes it over to roslaunch.
    Returns after roslaunch has terminated and temporary files have been removed.

    :param launch_obj: Instance of class launch.Launch
    :param dry_run: If Only print generated XML (default: False)
    :param silent: Hide roslaunch output
    """
    if silent:
        import sys
        import os
        sys.stdout = open(os.devnull, 'w')
    content = launch_obj.generate()
    if not dry_run:
        import tempfile
        import roslaunch  # dry-run even works w/o ROS
        ftmp = tempfile.NamedTemporaryFile(mode='w', suffix='.launch', delete=False)
        ftmp.write(content)
        ftmp.close()  # close it so that roslaunch can open it (file still exists)
        # noinspection PyBroadException
        try:
            on_initialize.fire()
            roslaunch.main(_strip_args(ftmp.name))  # actually do the launch!
        except Exception:
            pass
        utils.silent_remove(ftmp.name)
    else:
        print(content)
    on_terminate.fire()
    # Delete created (temporary) env-loader script files:
    Machine.cleanup()


def start_async(launch_obj, silent=False):
    """
    Call method start() in a separate process and returns without waiting for roslaunch to terminate. If p is the
    returned object, call roslaunch2.terminate(p) to shutdown roslaunch(2) and wait until roslaunch has terminated.

    :param launch_obj: Instance of class launch.Launch
    :param silent: Hide roslaunch output
    :return: Instance of class multiprocessing.Process
    """
    from multiprocessing import Process
    p = Process(target=start, args=(launch_obj, False, silent))
    p.start()
    return p


def terminate(instance):
    """
    Terminates the given roslaunch2 instance (a multiprocessing.Process instance) and waits until it has exited.

    :param instance: Object returned by start_async() to be terminated
    :return: None
    """
    if instance is None:
        return
    from multiprocessing import Process
    assert isinstance(instance, Process)
    import os
    import signal
    os.kill(instance.pid, signal.SIGINT)
    instance.join(10.0)
    instance.terminate()
    instance.join()


def main(command_line_args=None):
    """
    Defines the core logic (= Python based dynamic launch files) of roslaunch2. It does NOT create any
    launch modules or the like.

    :param command_line_args: List with command line arguments as strings
    :return: None
    """
    import os.path
    parser = _argument_parser()
    args, _ = parser.parse_known_args(args=command_line_args)
    init_logger(not args.no_colors)

    if len(args.launchfile) > 1:
        logger.warning("Multiple launch files at once are not supported (yet), just using the first.")

    args.launchfile = args.launchfile[0]

    # Add default extension:
    if not os.path.splitext(args.launchfile)[1]:
        args.launchfile += '.pyl'

    # ROS package name given? Try to resolve path to package and update <launchfile> path appropriately:
    if args.package:
        try:
            args.launchfile = Package(args.package).find(args.launchfile)
        except rospkg.ResourceNotFound as ex:
            critical("Launch module '{:s}' not found in package '{:s}', searched in: \n- {:s}".format(
                     args.launchfile, args.package, '\n- '.join(ex.ros_paths)))
            return

    # Import the launch module, generate the content, write it to a launch file and invoke 'roslaunch':
    try:
        m = Package.import_launch_module(args.launchfile)
    except ValueError:
        critical("Launch module '{:s}' not found.".format(args.launchfile))
        return

    # Roslaunch help text:
    if args.ros_args:
        m.main()
        sys.argv[0] = args.launchfile  # display correct script name
        parents = LaunchParameter.launch_parameter_list
        if len(parents) == 0:
            return
        elif len(parents) == 1:
            parser = parents[0]
        else:
            parser = LaunchParameter(description=parents[0].description,
                                     conflict_handler='resolve', parents=parents)
        sys.argv.append('--usage')  # force launch module to output its help text
        parser.add_argument('--usage', default=False, action='help', help=argparse.SUPPRESS)
        parser.parse_known_args()
        return

    launch_tree = m.main()
    start(launch_obj=launch_tree, dry_run=args.dry_run)
