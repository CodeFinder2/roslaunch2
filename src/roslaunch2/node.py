#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 13/03/2018

import warnings
import lxml.etree
import enum

import remapable
import interfaces
import package
import machine
import parameter
import environment
import random
import string


class Output(enum.IntEnum):
    """
    Possible output targets for ROS logging commands (e. g., ROS_INFO()).
    """
    Screen = 1
    Log = 2

    def __str__(self):
        if self.value == Output.Screen:
            return 'screen'
        elif self.value == Output.Log:
            return 'log'


class Runnable(remapable.Remapable, interfaces.Composable, interfaces.Composer):
    """
    Encapsulates the common attributes and methods of a <node> and <test> tag/class (internal base class).
    """
    def __init__(self, tag_name, pkg, node_type=None, name=None, args=None):
        """
        Initializes the Runnable object.

        :param tag_name: Name of XML tag
        :param pkg: Name of package containing the node/test; must be valid
        :param node_type: Type of the node/test (executable); will be set to pkg if None
        :param name: Optional name of the instance within roslaunch; will be set to node_type if None
        :param args: Optional command line arguments
        """
        remapable.Remapable.__init__(self)
        interfaces.Composable.__init__(self)
        interfaces.Composer.__init__(self, [parameter.Parameter, environment.EnvironmentVariable])
        self._pkg = package.Package(pkg) if type(pkg) is str else pkg
        if pkg and not node_type:
            node_type = str(self._pkg)
        if not name:
            name = Runnable.make_valid_base_name(node_type)
        if not pkg:
            raise ValueError("pkg='{}' cannot be empty or None.".format(pkg))
        self._node = node_type  # equals the 'type' attribute in XML
        self.name = Runnable.make_valid_base_name(name)
        self.args = args
        self.clear_params = None
        self.ns = None
        self.prefix = None  # equals the 'launch-prefix' attribute in XML
        self.__tag_name = tag_name

    @staticmethod
    def valid_base_name(name):
        """
        Tests whether the given "name" is conformant with the definition of a base name according to
        http://wiki.ros.org/Names .

        :param cls: Class instance
        :param name: Name to be tested (str)
        :return: True if valid, False otherwise
        """
        if len(name) == 0:
            return False
        if name.find('~') > -1 or name.find('/') > -1 or not name[0].isalpha():
            return False
        for ch in name:
            if not ch.isalnum() and ch != '_':
                return False
        return True

    @staticmethod
    def make_valid_base_name(name):
        """
        Replaces any invalid characters in the given (base) name with a underscore to make it valid. The first must be a
        letter which is selected randomly if not already. In case of an empty name, a random 10-element is returned.

        :param cls: Class instance
        :param name: Name to be fixed (str)
        :return: Patched/random base name (str), depending on the input
        """
        if len(name) == 0:
            import utils
            return utils.anon(10)
        res = []
        if not name[0].isalpha():
            res.append(random.choice(string.ascii_letters))
        for ch in name:
            if not ch.isalnum() and ch != '_':
                res.append('_')
            else:
                res.append(ch)
        return ''.join(res)

    @property
    def pkg(self):
        """
        Returns the associated package.

        :return: roslaunch2.package.Package instance
        """
        return self._pkg

    @property
    def node(self):
        """
        Node type (aka executable name).

        :return: String representing the node's type
        """
        return self._node

    @pkg.setter
    def pkg(self, value):
        """
        Allows to change the package, this Runnable belongs to.
        :param value: package.Package instance or ROS package name
        """
        if not value:
            raise ValueError("pkg='{}' cannot be empty or None.".format(value))
        self._pkg = package.Package(value) if type(value) is str else value

    @node.setter
    def node(self, value):
        """
        Allows to change the node executable name.
        :param value: New executable name

        """
        if not value:
            raise ValueError("node='{}' cannot be empty or None.".format(value))
        self._node = value

    def __del__(self):
        if not self.rooted:
            warnings.warn('{} has been created but never add()ed.'.format(str(self)), Warning, 2)

    def __str__(self):
        return '{:s}@{:s}: {:s}'.format(self.node, self.pkg, self.name)

    def add(self, param):
        """
        Adds the parameter to the node's parameter set.

        :param param: New parameter, can be any object derived from parameter.Parameter
        """
        for p in self.children:
            if param == p:
                raise ValueError("Parameter '{}' already added.".format(str(param)))

        interfaces.Composer.add(self, param)

    def clear_params(self, clear=None):
        """
        Sets the clear_params flag as provided by roslaunch, see, e. g., http://wiki.ros.org/roslaunch/XML/node
        :param clear: True to clear all parameter before the node is started
        """
        self.clear_params = clear

    def set_namespace(self, ns=None):
        """
        Sets the node's namespace. Can also be achieved using the Group class.

        :param ns: Name of namespace
        """
        self.ns = ns

    def debug(self, separate_window=True, auto_run=True):
        """
        Adds a gdb debug prefix to this node.
        See http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB

        :param separate_window: True to launch the node in a separate window using xterm
        :param auto_run: True to automatically start the node within gdb (otherwise, type "run" and press enter")
        """
        sw = 'xterm -e ' if separate_window else ''
        ar = '-ex run ' if auto_run else ''
        self.prefix = '{}gdb {}--args'.format(sw, ar)

    def generate(self, root, machines, pkg):
        """
        Appends the underlying roslaunch XML code to the given root object.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        """
        elem = lxml.etree.SubElement(root, self.__tag_name)
        remapable.Remapable.generate(self, elem, machines, self._pkg)

        interfaces.GeneratorBase.to_attr(elem, 'pkg', self._pkg, package.Package)
        interfaces.GeneratorBase.to_attr(elem, 'type', self._node, str)
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        interfaces.GeneratorBase.to_attr(elem, 'clear_params', self.clear_params, bool)
        interfaces.GeneratorBase.to_attr(elem, 'args', self.args, str)
        interfaces.GeneratorBase.to_attr(elem, 'ns', self.ns, str)
        interfaces.GeneratorBase.to_attr(elem, 'launch-prefix', self.prefix, str)
        return elem


class Node(Runnable):
    """
    For starting ROS nodes, equals <node>, see http://wiki.ros.org/roslaunch/XML/node
    """
    def __init__(self, pkg, node_type=None, name=None, output=Output.Screen, args=None):
        """
        Initializes the ROS node to be launched.

        :param pkg: Name of ROS package or package.Package instance
        :param node_type: Type of node executable; will be set to pkg.name() if empty
        :param name: Name of node instance when running; will be set to node_type
        :param output: Output flag to define where logging commands should be forwarded to
        :param args: Optional command line arguments
        """
        Runnable.__init__(self, 'node', pkg, node_type, name, args)
        assert not output or type(output) == Output
        self.output = output
        self._respawn = None  # None -> use roslaunch default
        self.respawn_delay = None
        self.machine = None
        self._required = None

    @property
    def required(self):
        """
        Retrieves the required flag.
        :return: True if node is required, False otherwise
        """
        return self._required

    @required.setter
    def required(self, value):
        """
        Sets the required flag, effectively (if True) requiring the node to be running. If the node dies, roslaunch will
        terminate the entire launch.

        :param value: New required flag
        """
        if value and self._respawn:
            raise ValueError('Cannot set both required and respawn to True (incompatible).')
        self._required = value

    @property
    def respawn(self):
        """
        Retrieves the respawn flag.
        :return: True if node is respawn when it terminates, False otherwise
        """
        return self._respawn

    @respawn.setter
    def respawn(self, value):
        """
        Sets the respawn flag, effectively (if True) restarting the node if killed.

        :param value: New respawn flag
        """
        if value and self._required:
            raise ValueError('Cannot set both required and respawn to True (incompatible).')
        self._respawn = value

    def start_on(self, machine_object):
        """
        Allows to start the node on a specific machine.

        :param machine_object: New machine object to be used for starting self remotely
        """
        self.machine = machine_object

    def generate(self, root, machines, pkg):
        """
        Appends the underlying roslaunch XML code to the given root object.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        """
        # If a set of machines was assigned to this node, it's now time to select the final
        # machine this node gets executed on.
        if isinstance(self.machine, machine.MachinePool):
            self.machine = self.machine.select()
        # The following allows machine.Resolvable objects in self.args:
        self.args = machine.Machine.resolve_if(self.args, self.machine, self._pkg)
        if self.args:
            self.args = str(self.args)
        elem = Runnable.generate(self, root, machines, pkg)
        interfaces.GeneratorBase.to_attr(elem, 'output', self.output, Output)
        interfaces.GeneratorBase.to_attr(elem, 'respawn', self._respawn, bool)
        interfaces.GeneratorBase.to_attr(elem, 'machine', self.machine, machine.Machine)
        if self._respawn:
            interfaces.GeneratorBase.to_attr(elem, 'respawn_delay', self.respawn_delay, str)
        interfaces.GeneratorBase.to_attr(elem, 'required', self._required, bool)
        if self.machine:
            assert type(machines) is list
            machines.append(self.machine)
        # Generate parameter/environment tags and resolve paths / environment variables:
        for p in self.children:
            p.generate(elem, self.machine, self._pkg)
