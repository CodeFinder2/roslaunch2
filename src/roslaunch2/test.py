import interfaces
import node


class Test(node.Runnable):
    """
    For starting ROS nodes as tests (see rostest), equals <test>.
    """
    def __init__(self, pkg, node_type, test_name, args=None, name=None):
        node.Runnable.__init__(self, 'test', pkg, node_type, test_name if name is None else name, args)
        self.test_name = test_name
        self.retry = None
        self.time_limit = None
        self.cwd = None

    def generate(self, root, machines, pkg):
        elem = node.Runnable.generate(self, root, machines, pkg)

        interfaces.GeneratorBase.to_attr(elem, 'test-name', self.test_name, str)
        interfaces.GeneratorBase.to_attr(elem, 'retry', self.retry, int)
        interfaces.GeneratorBase.to_attr(elem, 'time-limit', self.time_limit, float)
        interfaces.GeneratorBase.to_attr(elem, 'cwd', self.cwd, str)
        for p in self.children:  # generate parameters
            p.generate(elem, None)
