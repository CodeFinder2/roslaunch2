import lxml.etree
import warnings
import argparse
import sys

import interfaces
import enum


class LaunchParameter(argparse.ArgumentParser):
    def __init__(self, prog=None, usage=None, description=None, epilog=None, version=None,
                 parents=[], formatter_class=argparse.HelpFormatter, prefix_chars='-',
                 fromfile_prefix_chars=None, argument_default=None, conflict_handler='error',
                 add_help=True):
        argparse.ArgumentParser.__init__(self, prog, usage, description, epilog, version, parents,
                                         formatter_class, prefix_chars, fromfile_prefix_chars,
                                         argument_default, conflict_handler, add_help)

    def get_args(self):
        args, unknown = self.parse_known_args()
        # Remove our arguments (defined above) so that both the launch module as well as roslaunch
        # don't bother about it:
        sys.argv = sys.argv[:1] + unknown
        return args


class Parameter(interfaces.GeneratorBase):
    """
    Base class for parameters
    """
    def __init__(self):
        interfaces.GeneratorBase.__init__(self)
        self.rooted = False  # True if object has been add()ed to a parent

    def __del__(self):
        if not self.rooted:
            warnings.warn("'{}' parameter has been created but never add()ed."
                          .format(str(self)), Warning, 2)

    def generate(self, root, machines):
        raise NotImplementedError('generate() not implemented in "{}" yet.'.format(self.__class__.__name__))


class ServerParameter(Parameter):
    """
    For setting parameters on the ROS parameter server. Equals the <param> tag.
    """
    def __init__(self, name, value, textfile=None, binfile=None, command=None):
        Parameter.__init__(self)
        # TODO add checks for consistency
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
        Convenience method.
        """
        if name in kwargs:   # get value from dict item named 'name' as well
            return ServerParameter(name, kwargs[name])
        else:  # escalate to default value
            return ServerParameter(name, default)

    def generate(self, root, machine):
        elem = lxml.etree.SubElement(root, 'param')
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        # Python type of self.value varies/depends on input:
        interfaces.GeneratorBase.to_attr(elem, 'value', self.value)
        interfaces.GeneratorBase.to_attr(elem, 'type', ServerParameter.type_to_str(self.value))


class FileCommand(enum.IntEnum):
    """
    Defines file operations / commands to be used with FileParameter.
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
    Equals the <rosparam> tag.
    """
    def __init__(self, value=None, command=None, file_path=None, param=None, ns=None):
        Parameter.__init__(self)
        # TODO test for valid/consistency (valid combination in particular)
        self.command = command
        self.file_path = file_path
        self.param = param
        self.value = value
        self.ns = ns

    def __eq__(self, other):
        if not isinstance(other, self.__class__):  # different types are never equal
            return False
        return self.command == other.command and self.file_path == other.file_path and \
               self.param == other.param and self.value == other.value and self.ns == other.ns

    def generate(self, root, machine):
        elem = lxml.etree.SubElement(root, 'rosparam')
        interfaces.GeneratorBase.to_attr(elem, 'command', self.command, FileCommand)
        interfaces.GeneratorBase.to_attr(elem, 'param', self.param, str)
        interfaces.GeneratorBase.to_attr(elem, 'file', self.file_path, str)
        interfaces.GeneratorBase.to_attr(elem, 'ns', self.ns, str)
        if self.value:
            elem.text = self.value
