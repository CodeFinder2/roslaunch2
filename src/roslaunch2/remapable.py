from lxml import etree

from interfaces import GeneratorBase


class Remapable(GeneratorBase):
    """
    Represents the capability of being <remap>able.
    """
    def __init__(self):
        GeneratorBase.__init__(self)
        self.from_name = list()
        self.to_name = list()

    def remap(self, from_name, to_name):
        if from_name and to_name:
            self.from_name.append(from_name)
            self.to_name.append(to_name)
        else:
            raise ValueError('Parameters cannot be empty.')

    def unmap(self):
        self.from_name = list()
        self.to_name = list()

    def generate(self, root, machines):
        if self.from_name and self.to_name:
            for fn, tn in zip(self.from_name, self.to_name):
                elem = etree.SubElement(root, 'remap')
                GeneratorBase.to_attr(elem, 'from', fn, str)
                GeneratorBase.to_attr(elem, 'to', tn, str)
