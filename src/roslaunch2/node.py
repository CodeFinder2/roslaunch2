import warnings
import lxml.etree
import enum

import remapable
import interfaces
import package
import machine
import parameter


class Output(enum.IntEnum):
    Screen = 1
    Log = 2

    def __str__(self):
        if self.value == Output.Screen:
            return 'screen'
        elif self.value == Output.Log:
            return 'log'


class Node(remapable.Remapable, interfaces.Composable, interfaces.Composer):
    """
    For starting ROS nodes, equals <node>.
    """
    def __init__(self, pkg, node=None, name=None, output=Output.Screen, args=None):
        remapable.Remapable.__init__(self)
        interfaces.Composable.__init__(self)
        interfaces.Composer.__init__(self, [parameter.Parameter])
        if pkg and not node:
            node = pkg
        if not name:
            name = node
        if not pkg:
            raise ValueError("pkg='{}' cannot be empty or None.".format(pkg))
        self._pkg = package.Package(pkg) if type(pkg) is str else pkg
        self._node = node  # equals the 'type' attribute in XML
        self.name = name
        assert not output or type(output) == Output
        self.output = output
        self.args = args
        self._respawn = None  # None -> use roslaunch default
        self.machine = None
        self.respawn_delay = None
        self._required = None
        self.clear_params = None
        self.prefix = None  # equals the 'launch-prefix' attribute in XML
        self.ns = None
        self.rooted = False  # True if object has been add()ed to a parent

    @property
    def required(self):
        return self._required

    @required.setter
    def required(self, value):
        if value and self._respawn:
            raise ValueError('Cannot set both required and respawn to True (incompatible).')
        self._required = value

    @property
    def respawn(self):
        return self._respawn

    @respawn.setter
    def respawn(self, value):
        if value and self._required:
            raise ValueError('Cannot set both required and respawn to True (incompatible).')
        self._respawn = value

    @property
    def pkg(self):
        return self._pkg

    @property
    def node(self):
        return self._node

    @pkg.setter
    def pkg(self, value):
        if not value:
            raise ValueError("pkg='{}' cannot be empty or None.".format(value))
        self._pkg = package.Package(value) if type(value) is str else value

    @node.setter
    def node(self, value):
        if not value:
            raise ValueError("node='{}' cannot be empty or None.".format(value))
        self._node = value

    def __del__(self):
        if not self.rooted:
            warnings.warn('{} has been created but never add()ed.'.format(str(self)), Warning, 2)

    def __str__(self):
        return '{:s}@{:s}: {:s}'.format(self.node, self.pkg, self.name)

    def clear_params(self, clear=None):
        self.clear_params = clear

    def set_namespace(self, ns=None):
        self.ns = ns

    def add(self, param):
        for p in self.children:
            if param == p:
                raise ValueError("Parameter '{}' already added.".format(str(param)))

        interfaces.Composer.add(self, param)

    def start_on(self, machine_object):
        self.machine = machine_object

    def generate(self, root, machines):
        elem = lxml.etree.SubElement(root, 'node')
        remapable.Remapable.generate(self, elem, machines)

        interfaces.GeneratorBase.to_attr(elem, 'pkg', self._pkg, package.Package)
        interfaces.GeneratorBase.to_attr(elem, 'type', self._node, str)
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        interfaces.GeneratorBase.to_attr(elem, 'output', self.output, Output)
        interfaces.GeneratorBase.to_attr(elem, 'args', self.args, str)
        interfaces.GeneratorBase.to_attr(elem, 'respawn', self._respawn, bool)
        interfaces.GeneratorBase.to_attr(elem, 'machine', self.machine, machine.Machine)
        if self._respawn:
            interfaces.GeneratorBase.to_attr(elem, 'respawn_delay', self.respawn_delay, str)
        interfaces.GeneratorBase.to_attr(elem, 'required', self._required, bool)
        interfaces.GeneratorBase.to_attr(elem, 'clear_params', self.clear_params, bool)
        interfaces.GeneratorBase.to_attr(elem, 'ns', self.ns, str)
        interfaces.GeneratorBase.to_attr(elem, 'launch-prefix', self.prefix, str)
        if self.machine:
            assert type(machines) is list
            machines.append(self.machine)
        for p in self.children:  # generate parameters
            p.generate(elem, None)
