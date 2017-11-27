#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 27/11/2017

# Import all submodules typically used in launch modules:
from group import *
from launch import *
from node import *
from parameter import *
from machine import *
from environment import *
from test import *
from package import *
from logger import *
from utils import *
from remote import *

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


def _add_env_to_nodes(composer, environment_variable_dict={}):
    """
    Traverses the whole tree of the composer object and search for instances of environment.EnvironmentVariable.
    Copy each EnvironmentVariable to all subsequent node.Node instances that has not defined it on their own.
    :param composer: Instance of interfaces.Composer
    :param environment_variable_dict: Dictionary of instances of environment.EnvironmentVariable that are valid for
                                      composer (dictionary keys are the environment variable names)
    """
    from interfaces import Composer

    # Copy dict to not change the argument in higher recursion levels:
    tmp_env_dict = dict(environment_variable_dict)

    # Iterate over all children:
    for child in list(composer.children):
        if isinstance(child, EnvironmentVariable):
            # Found new environment variable; add or replace instances from higher levels:
            tmp_env_dict[child.name] = child
            # Remove environment variables from groups because in roslaunch groups are not allowed to have env members:
            # http://wiki.ros.org/roslaunch/XML/env
            if isinstance(composer, Group):
                composer.children.remove(child)

        elif isinstance(child, Node):
            # Found node; add previously found environment variables:
            for env_name in tmp_env_dict:
                env = tmp_env_dict[env_name]
                # Check if node already has defined an EnvironmentVariable with the same name:
                add_env = True
                for node_child in child.children:
                    if isinstance(node_child, EnvironmentVariable) and node_child.name == env_name:
                        add_env = False
                        break
                if add_env:
                    child += env

        elif isinstance(child, Composer):
            # Recursive call on composer children (launch, group, not node because handled earlier):
            _add_env_to_nodes(composer=child, environment_variable_dict=tmp_env_dict)


def _argument_parser(parents=[]):
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


def main():
    """
    Defines the core logic (= Python based dynamic launch files) of roslaunch2. It does NOT create any
    launch modules or the like.
    :return: None
    """
    import os.path
    parser = _argument_parser()
    # TODO: this does not "collect" all outputs of all (included/used) launch modules, just the first is shown
    args, _ = parser.parse_known_args()
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
        except IOError:
            critical("Launch module '{:s}' not found in package '{:s}'.".format(
                     args.launchfile, args.package))
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
    _add_env_to_nodes(composer=launch_tree)
    content = launch_tree.generate()
    if not args.dry_run:
        import tempfile
        import roslaunch  # dry-run even works w/o ROS
        ftmp = tempfile.NamedTemporaryFile(mode='w', suffix='.launch', delete=False)
        ftmp.write(content)
        ftmp.close()  # close it so that roslaunch can open it (file still exists)
        # noinspection PyBroadException
        try:
            on_initialize.fire()
            roslaunch.main(_strip_args(ftmp.name)) # actually do the launch!
        except:
            pass
        utils.silent_remove(ftmp.name)
    else:
        print(content)
    on_terminate.fire()
    # Delete created (temporary) env-loader script files:
    Machine.cleanup()
