import lxml.etree
import warnings
import argparse

import interfaces
import enum


class LaunchParameter(argparse.ArgumentParser):
    def __init__(self, prog=None, description=None, epilog=None, version=None,
                 parents=[], formatter_class=argparse.HelpFormatter, prefix_chars='-',
                 fromfile_prefix_chars=None, argument_default=None, conflict_handler='error'):
        argparse.ArgumentParser.__init__(self, prog, str(), description, epilog, version, parents,
                                         formatter_class, prefix_chars, fromfile_prefix_chars,
                                         argument_default, conflict_handler, add_help=False)
        self.add_argument('--usage', default=False, action='help', help=argparse.SUPPRESS)

    def add(self, name, help_text, default, **kwargs):
        """
        Generates a command line option for the current launch module named --name with the given help_text.
        Additionally, the default value is retrieved from kwargs[name] if that key exists. If not, the provided default
        value is set. This way, command line options have the highest precedence, followed by parameters passed by the
        kwargs parameter of a launch module's main() function. If neither of which are set, the provided default value
        is set.
        :param name: command line parameter name and key to retrieve (fallback) default in kwargs
        :param help_text: help text of command line option
        :param default: final default value if neither a command line argument nor the key in kwargs is given
        :param kwargs: dictionary of parameters for the launch module, possibly containing name
        :return: None
        """
        self.add_argument('--' + name, default=kwargs[name] if name in kwargs else default, help=help_text,
                          type=type(default))

    def add_flag(self, name, help_text, default, store, **kwargs):
        self.add_argument('--' + name, action='store_true' if store else 'store_false',
                          default=kwargs[name] if name in kwargs else default, help=help_text)

    def get_args(self):
        """
        Parse the previously defined command line arguments using add() or add_argument(). Ignored unknown args.
        Parameters are always parsed from sys.argv and may overlap with parameters from other (used / included) launch
        modules and/or with arguments of roslaunch.
        :return: detected / known arguments. If an argument is named --name, then args.name contains the value whereby
        args is the value returned by this method
        """
        known_args, _ = self.parse_known_args()
        return known_args


class Parameter(interfaces.GeneratorBase, interfaces.Composable):
    """
    Base class for parameters
    """
    def __init__(self):
        interfaces.GeneratorBase.__init__(self)
        interfaces.Composable.__init__(self)
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
        assert not command or type(command) == FileCommand
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
