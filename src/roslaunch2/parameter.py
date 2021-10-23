#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 08/06/2020

import lxml.etree
import warnings
import argparse
import os.path
import yaml
import enum

from . import interfaces
from . import machine
from . import logger


def load_from_file(path, only_parse_known_args):
    """
    Loads parameters from the given YAML file.

    :param path: Prefix to given file name argument
    :param only_parse_known_args: If True unknown arguments from yaml file are ignored,
                                  otherwise parsing fails with unknown arguments in yaml file
    """

    class LoadFromFile(argparse.Action):
        def __call__(self, parser, namespace, values, option_string=None):
            assert (len(values) == 1 and values[0]), "Invalid file name argument."

            filename = values[0]
            if not filename.endswith('.yaml'):
                filename = "{:s}.yaml".format(values[0])
            filepath = os.path.join(path, filename)
            try:
                f = yaml.safe_load(file(filepath, 'r'))
                for key, value in f.iteritems():
                    if value is not None:
                        if only_parse_known_args:
                            parser.parse_known_args(['--{:s}'.format(key), str(value)], namespace)
                        else:
                            parser.parse_args(['--{:s}'.format(key), str(value)], namespace)
            except yaml.YAMLError:
                logger.error("Cannot load parameters from file '{:s}'.".format(filename))

    return LoadFromFile


class LaunchParameter(argparse.ArgumentParser):
    """
    Represents a parameter for a launch module. For example, this can influence whether to select simulator A or B.
    These parameters are NOT consumed by ROS nodes (refer to ``ServerParameter`` and ``FileParameter`` in such cases).
    Such parameters are passed by command line, for example: roslaunch2 my_pkg my_launch.pyl --param foo

    Whats special about this class is that all command line parameters from all (possibly included) launch modules are
    added so that calling roslaunch2 with the special command line flag "--ros-args" prints all available parameters
    for the given launch file along with a description of it (like roslaunch does). In order to make this work, you
    must finally call 'get_args()'.
    """
    launch_parameter_list = []  # Static list collection all LaunchParameter instances.

    def __init__(self, prog=None, description=None, epilog=None,
                 parents=None, formatter_class=argparse.HelpFormatter, prefix_chars='-',
                 fromfile_prefix_chars=None, argument_default=None, conflict_handler='resolve'):
        if parents is None:
            parents = []
        argparse.ArgumentParser.__init__(self, prog=prog, usage=None, description=description, epilog=epilog,
                                         parents=parents, formatter_class=formatter_class, prefix_chars=prefix_chars,
                                         fromfile_prefix_chars=fromfile_prefix_chars, argument_default=argument_default,
                                         conflict_handler=conflict_handler, add_help=False)
        self.ros_argument_group = self.add_argument_group(title='ROS launch module arguments', description=None)

    def add(self, name, help_text, default, short_name=None, **kwargs):
        """
        Generates a command line option for the current launch module named ``--name`` with the given ``help_text``.
        Additionally, the default value is retrieved from ``kwargs[name]`` if that key exists. If not, the provided
        default  value is set. This way, command line options have the highest precedence, followed by parameters passed
        by the kwargs parameter of a launch module's ``main()`` function. If neither of which are set, the provided
        default value is set.

        :param name: command line parameter name and key to retrieve (fallback) default in kwargs
        :param help_text: help text of command line option
        :param default: final default value if neither a command line argument nor the key in kwargs is given
        :param short_name: Optional string: Short command line parameter name
        :param kwargs: dictionary of parameters for the launch module, possibly containing name
        :return: None
        """
        if short_name:
            if len(short_name) == 1:
                sn = '-' + short_name
            else:
                sn = '--' + short_name
            self.ros_argument_group.add_argument(sn, '--' + name, dest=name,
                                                 default=kwargs[name] if name in kwargs else default,
                                                 help=help_text, type=type(default))
        self.ros_argument_group.add_argument('--' + name, default=kwargs[name] if name in kwargs else default,
                                             help=help_text, type=type(default))

    def add_flag(self, name, help_text, default, store, short_name=None, **kwargs):
        """
        Generates a command line flag for the current launch module named ``--name`` with the given ``help_text``.
        Additionally, the default value is retrieved from ``kwargs[name]`` if that key exists. If not, the provided
        default value is set. This way, command line flags have the highest precedence, followed by parameters passed by
        the kwargs parameter of a launch module's ``main()`` function. If neither of which are set, the provided default
        value is set.

        :param name: command line parameter name and key to retrieve (fallback) default in kwargs
        :param help_text: help text of command line option
        :param default: final default value if neither a command line argument nor the key in kwargs is given
        :param store: ``True`` to store ``True`` of the flag is set / provided on the command line, ``False`` to store
               ``False`` if the flag is provided
        :param short_name: Optional string: Short command line parameter name
        :param kwargs: dictionary of parameters for the launch module, possibly containing name
        """
        if short_name:
            if len(short_name) == 1:
                sn = '-' + short_name
            else:
                sn = '--' + short_name
            self.ros_argument_group.add_argument(sn, '--' + name, dest=name,
                                                 action='store_true' if store else 'store_false',
                                                 default=kwargs[name] if name in kwargs else default,
                                                 help=help_text)
        self.ros_argument_group.add_argument('--' + name, action='store_true' if store else 'store_false',
                                             default=kwargs[name] if name in kwargs else default, help=help_text)

    def add_parameter_file(self, name, path, only_parse_known_args=False):
        """
        Generates a command line option named --name that loads parameters from a yaml file given by
        'path/value.yaml' where value is the actual command line value of ``--name``.

        :param name: Name of the command line option
        :param path: Path of the .yaml file
        :param only_parse_known_args: If True unknown arguments from yaml file are ignored
                                      otherwise parsing fails with unknown arguments in yaml file
        """
        self.ros_argument_group.add_argument('--' + name, action=load_from_file(path, only_parse_known_args),
                                             default='', nargs=1,
                                             help="Load parameters from file '<{:s}>.yaml'.".format(name.upper()))

    def get_args(self):
        """
        Parse the previously defined command line arguments using ``add()`` or ``add_argument()``. Ignores unknown
        arguments. Parameters are always parsed from ``sys.argv`` and may overlap with parameters from other
        (used / included) launch modules and/or with arguments of roslaunch.

        :return: detected / known arguments. If an argument is named ``--name``, then ``args.name`` contains the value
                 whereby ``args`` is the value returned by this method
        """
        self.launch_parameter_list.append(self)
        known_args, _ = self.parse_known_args()
        return known_args


class Parameter(interfaces.GeneratorBase, interfaces.Composable):
    """
    Base class for ROS node parameters.
    """

    def __init__(self):
        interfaces.GeneratorBase.__init__(self)
        interfaces.Composable.__init__(self)
        self.rooted = False  # True if object has been add()ed to a parent

    def __del__(self):
        if not self.rooted:
            warnings.warn("'{}' parameter has been created but never add()ed."
                          .format(str(self)), Warning, 2)


class ServerParameter(Parameter):
    """
    For setting parameters on the ROS parameter server. Equals the ``<param>`` tag.
    """
    def __init__(self, name, value, textfile=None, binfile=None, command=None):
        """
        Initializes the parameter object that is loaded to the ROS parameter server.

        :param name: Name of ROS parameter
        :param value: Value of the parameter
        :param textfile: Optional path to a text-based file whose content will be read and stored as a string
        :param binfile: Optional path to a binary file whose content will be read and stored as a base64-encoded XML-RPC
               binary object
        :param command: String of a command whose output will be read and stored as a string
        """
        Parameter.__init__(self)
        self.name = name
        self.value = value
        self.textfile = textfile
        self.binfile = binfile
        self.command = command

    def __eq__(self, other):
        if not isinstance(other, self.__class__):  # different types are never equal
            return False
        return self.name == other.name

    def __str__(self):
        return self.name

    @staticmethod
    def type_to_str(o):
        """
        Converts the Python type to the underlying roslaunch type string representation.

        :param o: Objects whose type should be converted
        :return: String representing the roslaunch type of o
        """
        if type(o) == str:
            return 'str'
        elif type(o) == int:
            return 'int'
        elif type(o) == float:
            return 'double'
        elif type(o) == bool:
            return 'bool'
        else:
            raise TypeError('{} (value: {}) not supported in roslaunch.'.format(type(o), o))

    @staticmethod
    def set(name, default, **kwargs):
        """
        Convenience method for setting a server parameter with an optional default, possibly extracting it from the
        provided dictionary kwargs.

        :param name: Name of parameter
        :param default: Default value for the parameter
        :param kwargs: additional arguments; if kwargs[name] exists, that value is used (otherwise, "default" is used)
        :return: parameter.ServerParameter instance
        """
        if name in kwargs:  # get value from dict item named 'name' as well
            return ServerParameter(name, kwargs[name])
        else:  # escalate to default value
            return ServerParameter(name, default)

    def generate(self, root, starting_machine, pkg):
        """
        Appends the underlying roslaunch XML code to the given root object.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        """
        elem = lxml.etree.SubElement(root, 'param')
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        self.value = machine.Machine.resolve_if(self.value, starting_machine, pkg)
        interfaces.GeneratorBase.to_attr(elem, 'value', self.value)
        interfaces.GeneratorBase.to_attr(elem, 'type', ServerParameter.type_to_str(self.value))


class FileCommand(enum.IntEnum):
    """
    Defines file operations / commands to be used with ``FileParameter``. This allows, e. g., to load a set of
    parameters from a given .yaml file.
    """
    Load = 1
    Dump = 2
    Delete = 3

    def __str__(self):
        if self.value == FileCommand.Load:
            return 'load'
        elif self.value == FileCommand.Dump:
            return 'dump'
        elif self.value == FileCommand.Delete:
            return 'delete'


class FileParameter(Parameter):
    """
    Used for loading, dumping or deleting YAML files to/from the ROS parameter server.
    Equals the ``<rosparam>`` tag.
    """

    def __init__(self, value=None, command=None, file_path=None, param=None, ns=None):
        Parameter.__init__(self)
        assert not command or type(command) == FileCommand
        self.command = command
        self.file_path = file_path
        self.param = param
        self.value = value
        self.ns = ns

    def __eq__(self, other):
        if not isinstance(other, self.__class__):  # different types are never equal
            return False
        return self.command == other.command and self.file_path == other.file_path and self.param == other.param and \
               self.value == other.value and self.ns == other.ns

    def generate(self, root, starting_machine, pkg):
        """
        Appends the underlying roslaunch XML code to the given root object.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        """
        elem = lxml.etree.SubElement(root, 'rosparam')
        interfaces.GeneratorBase.to_attr(elem, 'command', self.command, FileCommand)
        interfaces.GeneratorBase.to_attr(elem, 'param', self.param, str)
        interfaces.GeneratorBase.to_attr(elem, 'file', self.file_path, str)
        interfaces.GeneratorBase.to_attr(elem, 'ns', self.ns, str)
        if self.value:
            elem.text = self.value
