from lxml import etree

from interfaces import GeneratorBase


class Remapable(GeneratorBase):
    """
    Represents the capability of being <remap>able.
    """
    def __init__(self):
        GeneratorBase.__init__(self)
        self.from_name = None
        self.to_name = None

    def remap(self, from_name, to_name):
        self.from_name = from_name
        self.to_name = to_name

    def generate(self, root, machines):
        if self.from_name and self.to_name:
            elem = etree.SubElement(root, 'remap')
            GeneratorBase.to_attr(elem, 'from', self.from_name, str)
            GeneratorBase.to_attr(elem, 'to', self.to_name, str)
