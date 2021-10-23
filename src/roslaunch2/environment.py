#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 08/06/2020

import lxml.etree
import warnings

from . import interfaces
from . import machine


class EnvironmentVariable(interfaces.GeneratorBase, interfaces.Composable):
    """
    Represents an environment variable (<env> tag in roslaunch), see http://wiki.ros.org/roslaunch/XML/env
    This class cannot be used inside a Machine object (deprecated since ROS Fuerte). However, you can use this
    class to create remote env. variables by setting the "value" to an resolvable object, see remote.py. Also note that
    env. variables are being propagated to nodes if they are added to Groups or Launch objects, see
    Composer.add_env_variables_to_nodes() for more information. For instance:
    root += EnvironmentVariable('ROSCONSOLE_CONFIG_FILE', Path('/config/rosconsole.cfg', pkg))
    (assuming "root" is a "Launch" object), would cause the env. variable to be added to all nodes started afterwards.
    If one of these nodes is started on system A, and another one is launched on system B, the env. variable will be
    resolved remotely on theses systems A and B respectively (given the "pkg" where the path relates to).
    """
    def __init__(self, name, value):
        """
        Initializes the environment variable with the given name and value.

        :param name: Name of environment variable
        :param value: Actual value
        """
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
        """
        Generates the underlying roslaunch XML code.

        :param root: XML root element object
        :param starting_machine: May be set to a machine.Machine object where the environment variable should be
               resolved on
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        :return: None
        """
        elem = lxml.etree.SubElement(root, 'env')
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        if not isinstance(starting_machine, machine.Machine):
            starting_machine = machine.Localhost
        self.value = machine.Machine.resolve_if(self.value, starting_machine, pkg)
        interfaces.GeneratorBase.to_attr(elem, 'value', self.value)
