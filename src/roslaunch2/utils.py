#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 13/03/2018

import string
import random
import os
import itertools
import errno

#: Separator for communication (topic, services) and tf (frame) names
ROS_NAME_SEP = '/'


def anon(length=6):
    """
    Creates random ROS names.

    Generates a random ROS name (see also http://wiki.ros.org/Names). Clearly, the longer the
    name, the smaller the probability of a name collision.

    :param length: number of characters in the generated name
    :return: random name with [A-Z,a-z,0-9]
    """
    def id_gen(size, chars=string.ascii_uppercase + string.ascii_lowercase + string.digits):
        """
        Creates a random ROS name (e. g., valid for ROS nodes) with the given size and containing
        the specified set of characters.

        :param size: length of generated name
        :param chars: types of characters to use for generation
        :return: generated random name
        """
        return ''.join(random.choice(chars) for _ in range(size))

    #if length <= 1:
    #    raise NameError("ROS names should be longer than 1 character.")
    return "{0}{1}".format(id_gen(1, chars=string.ascii_uppercase + string.ascii_lowercase), id_gen(length - 1))


def silent_remove(path):
    """
    Remove the file given by path and do not fail if the file does not exist.

    :param path: path to file which should be deleted silently
    :return: Returns True if the file was removed, False otherwise.
    """
    if not path:
        return False
    try:
        os.remove(path)
        return True
    except OSError as ose:
        if ose.errno != errno.ENOENT:  # errno.ENOENT = no such file or directory
            raise  # re-raise exception if a different error occurred
        else:
            return False


def merge_dicts(x, y):
    """
    Given two dicts, merge them into a new dict as a shallow copy. Values in y that are also already present in x will
    overwrite them. This is supplied to be compatible with Python 2.x, see also
    http://stackoverflow.com/questions/38987/how-to-merge-two-python-dictionaries-in-a-single-expression

    :param x: first dict to merge
    :param y: second dict to merge
    :return: merged dictionary
    """
    z = x.copy()
    z.update(y)
    return z


def clean_name(ros_name, c=ROS_NAME_SEP):
    """
    Removes successive duplicates of c from the given ros_name.

    :param ros_name: Name (string) to process
    :param c: character whose successive occurrences should be removed
    :return: processed string (or unchanged ros_name if nothing needs to be done)
    """
    return ''.join(c if a == c else ''.join(b) for a, b in itertools.groupby(ros_name))


def tf_join(left, right):
    """
    Combines the partial frame IDs left and right to a new combined valid frame IDs. According to tf2
    design, preceding slashes will be stripped.

    :param left: partial frame ID to put leftmost (e. g., a parent frame ID)
    :param right: partial frame ID to put rightmost (e. g., a child frame ID)
    :return: combined frame ID (e. g., "parent1/child0")
    """
    # Join left and right, and get rid of successive occurrences of ROS_NAME_SEP:
    res = clean_name(ROS_NAME_SEP.join([left, right]), ROS_NAME_SEP)
    # Remove a possibly preceding ROS_NAME_SEP:
    return res[1:] if res.startswith(ROS_NAME_SEP) else res


def ros_join(left, right, force_global=False):
    """
    Behaves much like tf_join but is intended to work on ROS names (like topics, services, namespaces).
    Allows to force the creation of a global name.

    :param left: partial name to put leftmost
    :param right: partial name to put rightmost, can also be a list of names
    :param force_global: True to ensure / force the resulting name to be in the global namespace (not advised, but
                         useful/required in rare cases). If left is already global, the resulting name remains global.
    :return: combined ROS name
    """
    if type(right) is list or type(right) is tuple:
        res = clean_name(ROS_NAME_SEP.join([left] + right), ROS_NAME_SEP)
    else:
        res = clean_name(ROS_NAME_SEP.join([left, right]), ROS_NAME_SEP)
    return ROS_NAME_SEP + res if force_global and not res.startswith(ROS_NAME_SEP) else res


def to_ros_base_name(data, replacement_begin=anon(1), replacement_other='_'):
    """
    Converts the given data to a valid ROS base (!) name.
    :param data: string to be converted
    :param replacement_begin: Replacement string for the beginning (if a replacement is necessary)
    :param replacement_other: Replacement for subsequent invalid characters (if any)
    :return: Possibly modified valid ROS base name, given data as input
    """
    result = ""
    if not data[0].isalpha():
        result = replacement_begin
    else:
        result = data[0]
    for i in range(1, len(data)):
        if not data[i].isalnum():
            result += replacement_other
        else:
            result += data[i]
    return result


class Observable(object):
    """
    Allows to create observable events (like the termination of roslaunch) to register custom actions in case such
    events are being triggered.
    """
    def __init__(self):
        self.callbacks = []

    def subscribe(self, callback, **kwargs):
        """
        Allows to register a custom action to be executed if the associated event (= object of this class) fires.

        :param callback: Python function to be executed ("callback") when the event is being triggered
        :param kwargs: Optional additional parameters for the callback, may be None / ignored
        """
        self.callbacks.append((callback, kwargs))

    def fire(self):
        """
        Triggers the event (typically only used by the enitity defining the event, i. e., roslaunch2 here).
        """
        for fn, kwargs in self.callbacks:
            fn(**kwargs)
