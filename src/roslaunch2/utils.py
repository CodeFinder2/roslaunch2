import string
import random


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
