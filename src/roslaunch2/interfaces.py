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


class Composable(object):  # TODO: Composer vs. Composable
    def __init__(self):
        self.children = []

    def add(self, other):
        if hasattr(other, 'rooted'):
            other.rooted = True
        self.children.append(copy.deepcopy(other))
