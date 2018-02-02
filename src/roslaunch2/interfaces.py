#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 26/01/2018

import copy
import lxml.etree


class GeneratorBase(object):
    """
    Represents the interface for classes that can generate its XML representation for roslaunch. You need to implement
    the generate() method for this to work.
    """

    def __init__(self):
        """
        Does nothing.
        """
        pass

    @staticmethod
    def to_attr(elem, name, value, expected_type=None):
        """
        Creates an XML-based element from the given attribute name and assigned the given value (i. e.
        <foo name="value"/>). Optionally verifies value against the expected_type if not None. No attribute is create
        if the value is None in order to use roslaunch' default.

        :param elem: XML element where the attribute should be set
        :param name: Name of attribute
        :param value: Value of attribute
        :param expected_type: Expected type that value should have (None to disable this check, e. g., where the type
                              is not known a priori)
        :return: None
        """
        # None represents roslaunch's default value so let's omit the attribute:
        if value is not None:
            # noinspection PyProtectedMember
            assert isinstance(elem, lxml.etree._Element)
            assert isinstance(name, str)
            if expected_type:
                assert isinstance(value, expected_type)
            data = str(value)
            if type(value) == bool:
                data = data.lower()
            elem.set(name, data)

    def generate(self, root, machines, pkg):  # generates XML output
        """
        Generates a roslaunch compatible XML representation of this object.
        :return: the return value can be used arbitrarily (for an example, see node.Runnable); however, finally, a
        string (see Launch impl.) must be returned which is the XML representation of the roslaunch file
        """
        raise NotImplementedError('generate() not implemented in "{}" yet.'.format(self.__class__.__name__))


class Composable(object):
    """
    Represents the interface for objects of classes that can be added to Composer instances. They remember if they have
    already been added to show a warning about created but unused objects. An example is the ServerParameter class which
    cannot contain any other core type of the roslaunch2 hierarchy.
    """

    def __init__(self):
        """
        Default-constructs the object.
        """
        self.rooted = False  # True if object has been (Composer.)add()ed to a parent


class Composer(object):
    """
    Represents the interface for objects of classes allowing to contain other objects. An example is the Node class
    which can contain Parameter objects.
    """

    def __init__(self, valid_types):
        """
        Constructs the object.

        :param valid_types: tuple-convertible list of types that are allowed to be added to this composer
        """
        self.children = []
        self.valid_types = tuple(valid_types) if valid_types is not None else None

    def add(self, other):
        """
        Allows to compose objects of type Composable by adding them to this internal list. This is used to create a
        hierarchy of objects. For example, a Launch object can contain a Group. Both are Composer objects. It throws an
        assertion if you try to add objects that are not allowed (can be defined in the constructor).

        Note that objects are deep-copied so that they can be altered after adding them without chaning the added
        object.

        :param other: Object to be added to this composer
        :return: None
        """
        assert isinstance(other, Composable), "Object to be added must have the 'Composable' base class."
        if self.valid_types is not None:
            assert isinstance(other, self.valid_types),\
                "Cannot add objects of type '{:s}' to '{:s}'.".format(other.__class__.__name__,
                                                                      self.__class__.__name__)
        other.rooted = True
        self.children.append(copy.deepcopy(other))

    def __iadd__(self, other):
        """
        Behaves like add().

        :param other: Object to be added to this composer
        :return: a reference to itself (self)
        """
        self.add(other)
        return self

    def add_env_variables_to_nodes(self, environment_variable_dict=None):
        """
        Traverses the whole tree of this composer object and searches for instances of environment.EnvironmentVariable.
        Copy each EnvironmentVariable to all subsequent node.Node instances that has not defined it on their own.

        :param environment_variable_dict: Dictionary of instances of environment.EnvironmentVariable that are valid for
                                          composer (dictionary keys are the environment variable names)
        """
        if environment_variable_dict is None:
            environment_variable_dict = {}
        from environment import EnvironmentVariable
        from group import Group
        from node import Node

        # Copy dict to not change the argument in higher recursion levels:
        tmp_env_dict = dict(environment_variable_dict)

        # Iterate over all children:
        for child in list(self.children):
            if isinstance(child, EnvironmentVariable):
                # Found new environment variable; add or replace instances from higher levels:
                tmp_env_dict[child.name] = child
                # Remove environment variables from groups cause roslaunch groups are not allowed to have env members:
                # http://wiki.ros.org/roslaunch/XML/env
                if isinstance(self, Group):
                    self.children.remove(child)

            elif isinstance(child, Node):
                # Found node; add previously found environment variables:
                for env_name in tmp_env_dict:
                    env = tmp_env_dict[env_name]
                    # Check if node already has defined an EnvironmentVariable with the same name:
                    add_env = True
                    for node_child in child.children:
                        if isinstance(node_child, EnvironmentVariable) and node_child.name == env_name:
                            add_env = False
                            break
                    if add_env:
                        child += env

            elif isinstance(child, Composer):
                # Recursive call on composer children (launch, group, not node because handled earlier):
                child.add_env_variables_to_nodes(environment_variable_dict=tmp_env_dict)
