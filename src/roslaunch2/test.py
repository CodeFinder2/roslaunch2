#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 13/03/2018

import interfaces
import node


class Test(node.Runnable):
    """
    For starting ROS nodes as tests (see rostest), equals <test>.
    """
    def __init__(self, pkg, node_type, test_name, args=None, name=None):
        """
        Initializes the test instance.

        :param pkg: Name of ROS package
        :param node_type: Name of test executable
        :param test_name: Name of the est
        :param args: Optional command line arguments
        :param name: Name of the node instance
        """
        node.Runnable.__init__(self, 'test', pkg, node_type, test_name if name is None else name, args)
        self.test_name = test_name
        self.retry = None
        self.time_limit = None
        self.cwd = None

    def generate(self, root, machines, pkg):
        """
        Appends the underlying roslaunch XML code to the given root object.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        """
        elem = node.Runnable.generate(self, root, machines, pkg)

        interfaces.GeneratorBase.to_attr(elem, 'test-name', self.test_name, str)
        interfaces.GeneratorBase.to_attr(elem, 'retry', self.retry, int)
        interfaces.GeneratorBase.to_attr(elem, 'time-limit', self.time_limit, float)
        interfaces.GeneratorBase.to_attr(elem, 'cwd', self.cwd, str)
        for p in self.children:  # generate parameters
            p.generate(elem, None)
