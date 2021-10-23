#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 08/06/2020

from lxml import etree

from . import interfaces


class Remapable(interfaces.GeneratorBase):
    """
    Represents the capability of being <remap>able.
    """
    def __init__(self):
        interfaces.GeneratorBase.__init__(self)
        self.from_name = list()
        self.to_name = list()

    def remap(self, from_name, to_name):
        """
        Remap the ROS name from_name (e. g., a topic) to the new name to_name.

        :param from_name: Previous name
        :param to_name: New ROS name
        :return: None
        """
        if from_name and to_name:
            self.from_name.append(from_name)
            self.to_name.append(to_name)
        else:
            raise ValueError('Parameters cannot be empty.')

    def unmap(self):
        """
        Clear all remappings added by remap().

        :return: None
        """
        self.from_name = list()
        self.to_name = list()

    def generate(self, root, machines, pkg):
        """
        Appends the underlying roslaunch XML code to the given root object.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        """
        if self.from_name and self.to_name:
            for fn, tn in zip(self.from_name, self.to_name):
                elem = etree.SubElement(root, 'remap')
                interfaces.GeneratorBase.to_attr(elem, 'from', fn, str)
                interfaces.GeneratorBase.to_attr(elem, 'to', tn, str)
