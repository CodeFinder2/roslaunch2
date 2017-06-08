import lxml.etree
import warnings

import interfaces
import machine


class EnvironmentVariable(interfaces.GeneratorBase, interfaces.Composable):
    """
    Note: this class cannot be used inside a Machine object (deprecated since ROS Fuerte).
    """
    def __init__(self, name, value):
        interfaces.GeneratorBase.__init__(self)
        interfaces.Composable.__init__(self)
        self.name = name
        self.value = value
        warnings.warn("EnvironmentVariable (<env> tag) is deprecated since Fuerte. Use env-loader instead.",
                      DeprecationWarning)

    def __del__(self):
        if not self.rooted:
            warnings.warn('{} has been created but never add()ed.'.format(str(self)), Warning, 2)

    def generate(self, root, starting_machine, pkg):
        elem = lxml.etree.SubElement(root, 'env')
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        if not isinstance(starting_machine, machine.Machine):
            starting_machine = machine.Localhost
        self.value = machine.Machine.resolve_if(self.value, starting_machine, pkg)
        interfaces.GeneratorBase.to_attr(elem, 'value', self.value)
