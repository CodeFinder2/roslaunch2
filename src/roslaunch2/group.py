import lxml.etree
import warnings

import interfaces
import remapable
import node
import launch


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
        self.machine = None

    def __del__(self):
        if not self.rooted:
            warnings.warn('{} has been created but never add()ed.'.format(str(self)), Warning, 2)

    def start_on(self, machine_object):
        self.machine = machine_object

    def generate(self, root, machines):
        if self.name:  # exclude the group if namespace is empty
            elem = lxml.etree.SubElement(root, 'group')
            interfaces.GeneratorBase.to_attr(elem, 'ns', self.name)
            interfaces.GeneratorBase.to_attr(elem, 'clear_params', self.clear_params)
            remapable.Remapable.generate(self, elem, machines)
        else:
            elem = root
        # Do not ignore the content:
        if not self.ignore_content:
            for child in self.children:
                # For all nodes or groups in this group, start them on the machine if a machine has been given for this
                # group and they do not have a machine assigned yet. Treating groups and nodes equally, we can propagate
                # machines to nodes nested in sub(-sub(-sub(-...)))groups of this one.
                if self.machine and (isinstance(child, node.Node) or isinstance(child, Group) or
                   isinstance(child, launch.Launch)) and not child.machine:
                    child.start_on(self.machine)
                child.generate(elem, machines)
