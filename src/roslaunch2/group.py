import lxml.etree
import interfaces
import remapable
import warnings


class Group(remapable.Remapable, interfaces.Composable, interfaces.GeneratorBase):
    """
    For grouping nodes in namespaces, equals <group ns="namespace"> ... </group>.
    """
    def __init__(self, namespace, ignore_content=False, clear_params=None):
        remapable.Remapable.__init__(self)
        interfaces.Composable.__init__(self)
        self.name = namespace
        self.clear_params = clear_params
        self.ignore_content = ignore_content
        self.rooted = False  # True if object has been add()ed to a parent

    def __del__(self):
        if not self.rooted:
            warnings.warn('{} has been created but never add()ed.'.format(str(self)), Warning, 2)

    def generate(self, root, machines):
        if self.name:  # exclude the group if namespace is empty
            elem = lxml.etree.SubElement(root, 'group')
            interfaces.GeneratorBase.to_attr(elem, 'ns', self.name)
            interfaces.GeneratorBase.to_attr(elem, 'clear_params', self.clear_params)
            remapable.Remapable.generate(self, elem, machines)
        # Do not ignore the content:
        if not self.ignore_content:
            if not self.name:
                elem = root
            for child in self.children:
                child.generate(elem, machines)
