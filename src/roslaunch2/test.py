import lxml.etree
import warnings

import remapable
import interfaces
import package
import utils
import parameter


# TODO implement this completely
class Test(remapable.Remapable, interfaces.Composable, interfaces.Composer):
    """
    For starting ROS nodes as tests (see rostest), equals <test>.
    """
    def __init__(self, pkg, node, test_name=None, args=None, name=None):
        remapable.Remapable.__init__(self)
        interfaces.Composable.__init__(self)
        interfaces.Composer.__init__(self, [parameter.Parameter])
        if not pkg or not node:
            raise ValueError("pkg='{}' and/or node='{}' cannot be empty or None.".format(pkg, node))
        self.pkg = package.Package(pkg) if type(pkg) is str else pkg
        self.node = node  # equals the 'type' attribute in XML
        self.test_name = test_name
        self.name = name if name else utils.anon()
        self.args = args
        self.clear_params = None
        self.prefix = None  # equals the 'launch-prefix' attribute in XML
        self.ns = None
        self.retry = None
        self.time_limit = None
        self.cwd = None
        self.rooted = False  # True if object has been add()ed to a parent

    def __del__(self):
        if not self.rooted:
            warnings.warn('{} has been created but never add()ed.'.format(str(self)), Warning, 2)

    def clear_params(self, clear=None):
        self.clear_params = clear

    def set_namespace(self, ns=None):
        self.ns = ns

    def add(self, param):
        for p in self.children:
            if param == p:
                raise ValueError("Parameter '{}' already added.".format(str(param)))
        interfaces.Composer.add(self, param)

    def generate(self, root, machines):
        elem = lxml.etree.SubElement(root, 'test')
        remapable.Remapable.generate(self, elem, machines)

        interfaces.GeneratorBase.to_attr(elem, 'pkg', self.pkg, package.Package)
        interfaces.GeneratorBase.to_attr(elem, 'type', self.node, str)
        interfaces.GeneratorBase.to_attr(elem, 'test-name', self.test_name, str)
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        interfaces.GeneratorBase.to_attr(elem, 'args', self.args, str)
        interfaces.GeneratorBase.to_attr(elem, 'clear_params', self.clear_params, bool)
        interfaces.GeneratorBase.to_attr(elem, 'ns', self.ns, str)
        interfaces.GeneratorBase.to_attr(elem, 'launch-prefix', self.prefix, str)
        interfaces.GeneratorBase.to_attr(elem, 'retry', self.retry, int)
        interfaces.GeneratorBase.to_attr(elem, 'time-limit', self.time_limit, float)
        interfaces.GeneratorBase.to_attr(elem, 'cwd', self.cwd, str)
        for p in self.children:  # generate parameters
            p.generate(elem, None)
