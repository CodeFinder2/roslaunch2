import string
import random
import os
import itertools

# Separator for communication (topic, services) and tf (frame) names:
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

    if length <= 1:
        raise NameError("ROS names should be longer than 1 character.")
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


def tf_join(left, right):
    def remove_successive_duplicates(s, c):
        return ''.join(c if a == c else ''.join(b) for a, b in itertools.groupby(s))

    res = ROS_NAME_SEP.join([left, right])
    return remove_successive_duplicates(res, ROS_NAME_SEP)


def ros_join(left, right, force_global=False):
    return tf_join(ROS_NAME_SEP + left, right) if force_global else tf_join(left, right)
