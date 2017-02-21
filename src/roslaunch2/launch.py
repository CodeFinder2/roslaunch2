import warnings
import lxml.etree

import interfaces
import machine
import remapable
import node
import group


class Launch(interfaces.Composable, interfaces.Composer, remapable.Remapable):
    def __init__(self, deprecation_message=None):
        interfaces.Composable.__init__(self)
        interfaces.Composer.__init__(self, None)  # everything can be put into Launch
        remapable.Remapable.__init__(self)
        self.machine = None
        if deprecation_message:
            warnings.warn(deprecation_message, DeprecationWarning)

    def __repr__(self):
        return self.children.__repr__()

    def start_on(self, machine_object):
        self.machine = machine_object

    def add(self, other):
        if isinstance(other, machine.Machine):
            message = "Machines shouldn't be add()ed explicitly (skipping {})" \
                .format(repr(other))
            warnings.warn(message, Warning)
        else:
            super(Launch, self).add(other)

    def generate(self, root=None, machines=None):
        # Work around this issue (order of boolean expressions matters!):
        # http://stackoverflow.com/questions/20129996/why-does-boolxml-etree-elementtree-element-evaluate-to-false
        first_call = isinstance(root, type(None)) and not root
        if first_call:
            root = lxml.etree.Element('launch')
            machines = []
        for child in self.children:
            if self.machine and (isinstance(child, node.Node) or isinstance(child, group.Group) or
               isinstance(child, Launch)) and not child.machine:  # see comment in group.py
                child.start_on(self.machine)
            child.generate(root, machines)
        if first_call:
            # Machines currently insert themselves at the beginning (for simplicity):
            machines = list(set(machines))  # make unique based on machine names
            for m in machines:
                m.generate(root, None)
            return str(lxml.etree.tostring(root, pretty_print=True, xml_declaration=True,
                                           encoding='UTF-8', standalone='yes').decode("utf-8"))
