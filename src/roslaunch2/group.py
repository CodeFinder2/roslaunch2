#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 12/12/2021

import lxml.etree
import warnings

import roslaunch2.interfaces
import roslaunch2.remapable
import roslaunch2.node
import roslaunch2.launch


class Group(roslaunch2.remapable.Remapable, roslaunch2.interfaces.Composer, roslaunch2.interfaces.Composable):
    """
    For grouping nodes in namespaces, equals <group ns="namespace"> ... </group>.
    """
    def __init__(self, namespace, ignore_content=False, clear_params=None):
        """
        Initializes the Group object. If the namespace is empty or None, all elements of the group will appear in the
        resulting launch code but without a namespace (this is different to roslaunch but more flexible).

        :param namespace: Name of ROS namespace, can be empty
        :param ignore_content: Simple switch to not add the entire Group's content to the launch code (if set to True)
        :param clear_params: True to clear all parameters in the node's private namespace before launch, False to keep
               them unchanged
        """
        roslaunch2.remapable.Remapable.__init__(self)
        roslaunch2.interfaces.Composer.__init__(self, None)  # everything can be put into a Group
        roslaunch2.interfaces.Composable.__init__(self)
        self.name = namespace
        self.clear_params = clear_params
        self.ignore_content = ignore_content
        self.machine = None

    def __del__(self):
        if not self.rooted:
            warnings.warn('{} has been created but never add()ed.'.format(str(self)), Warning, 2)

    def start_on(self, machine_object):
        """
        Allows to start all nodes added to this group to be launched on the given machine. However, if a node has
        already been assigned a specific machine object, the given machine_object is ignored for that node.

        :param machine_object: Machine object to be used for launching remotely; a roslaunch2_server needs to be running
               on that machine
        """
        self.machine = machine_object

    def generate(self, root, machines, pkg):
        """
        Generates the underlying roslaunch XML code.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        """
        if self.name:  # exclude the group if namespace is empty
            elem = lxml.etree.SubElement(root, 'group')
            roslaunch2.interfaces.GeneratorBase.to_attr(elem, 'ns', self.name)
            roslaunch2.interfaces.GeneratorBase.to_attr(elem, 'clear_params', self.clear_params)
            roslaunch2.remapable.Remapable.generate(self, elem, machines, pkg)
        else:
            elem = root
        # Do not ignore the content:
        if not self.ignore_content:
            for child in self.children:
                # For all nodes or groups in this group, start them on the machine if a machine has been given for this
                # group and they do not have a machine assigned yet. Treating groups and nodes equally, we can propagate
                # machines to nodes nested in sub(-sub(-sub(-...)))groups of this one.
                if self.machine and (isinstance(child, roslaunch2.node.Node) or isinstance(child, Group) or
                   isinstance(child, roslaunch2.launch.Launch)) and not child.machine:
                    child.start_on(self.machine)
                child.generate(elem, machines, pkg)
