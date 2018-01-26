#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 26/01/2018

import warnings
import lxml.etree

import interfaces
import machine
import remapable
import node
import group


class Launch(interfaces.Composable, interfaces.Composer, remapable.Remapable):
    """
    Represents the root object of a launch module, similiar to roslaunch's <launch> tag.
    """
    def __init__(self, deprecation_message=None):
        interfaces.Composable.__init__(self)
        interfaces.Composer.__init__(self, None)  # everything can be put into Launch
        remapable.Remapable.__init__(self)
        self.machine = None
        if deprecation_message:
            warnings.warn(deprecation_message, DeprecationWarning)

    def __repr__(self):
        return self.children.__repr__()

    def start_on(self, machine_object):
        """
        Allows to start all nodes added to this launch object to be launched on the given machine. However, if a node
        has already been assigned a specific machine object, the given machine_object is ignored for that node. Also,
        groups with their own machine object will precede machine_object.

        :param machine_object: Machine object to be used for launching remotely; a roslaunch2_server needs to be running
               on that machine
        :return: None
        """
        self.machine = machine_object

    def add(self, other):
        """
        Allows to add other objects of type Launch, Group, Node, Test, etc. to the launch module.

        :param other: Object to be added
        :return: None
        """
        if isinstance(other, machine.Machine):
            message = "Machines shouldn't be add()ed explicitly (skipping {})" \
                .format(repr(other))
            warnings.warn(message, Warning)
        else:
            super(Launch, self).add(other)

    def generate(self, root=None, machines=None, pkg=None):
        """
        Generates the underlying roslaunch XML code.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        :return: string representation in UTF-8 encoding containing the generated XML code if generate() was called on
                 the top level (i. e., the Launch object was not nested as a part of anther launch module; in such cases
                 None is returned)
        """
        self.add_env_variables_to_nodes()
        # Work around this issue (order of boolean expressions matters!):
        # http://stackoverflow.com/questions/20129996/why-does-boolxml-etree-elementtree-element-evaluate-to-false
        first_call = isinstance(root, type(None)) and not root
        if first_call:
            root = lxml.etree.Element('launch')
            machines = []
        remapable.Remapable.generate(self, root, machines, None)
        for child in self.children:
            if self.machine and (isinstance(child, node.Node) or isinstance(child, group.Group) or
               isinstance(child, Launch)) and not child.machine:  # see comment in group.py
                child.start_on(self.machine)
            child.generate(root, machines, None)
        if first_call:
            # Machines currently insert themselves at the beginning (for simplicity):
            machines = list(set(machines))  # make unique based on machine names
            for m in machines:
                m.generate(root, None, None)
            return str(lxml.etree.tostring(root, pretty_print=True, xml_declaration=True,
                                           encoding='UTF-8', standalone='yes').decode("utf-8"))
