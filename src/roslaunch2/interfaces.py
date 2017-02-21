import copy
import lxml.etree


class GeneratorBase(object):  # 'base class'?
    def __init__(self):
        pass

    @staticmethod
    def to_attr(elem, name, value, expected_type=None):
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

    def generate(self, root, machines):  # generates XML output
        """
        Generates a roslaunch compatible XML representation of this object.
        :return: XML representation of type etree.Element
        """
        raise NotImplementedError('generate() not implemented in "{}" yet.'.format(self.__class__.__name__))


class Composable(object):
    def __init__(self):
        self.rooted = False  # True if object has been (Composer.)add()ed to a parent


class Composer(object):
    def __init__(self, valid_types):
        self.children = []
        self.valid_types = tuple(valid_types) if valid_types is not None else None

    def add(self, other):
        assert isinstance(other, Composable), "Object to be added must have the 'Composable' base class."
        if self.valid_types is not None:
            assert isinstance(other, self.valid_types),\
                "Cannot add objects of type '{:s}' to '{:s}'.".format(other.__class__.__name__,
                                                                      self.__class__.__name__)
        other.rooted = True
        self.children.append(copy.deepcopy(other))
