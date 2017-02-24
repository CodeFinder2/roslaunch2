import lxml.etree
import paramiko

import interfaces
import utils


class Machine(interfaces.GeneratorBase):
    """
    For defining (remote) machines to launch on, equals the <machine> tag.
    """
    def __init__(self, address, user, env_loader=None, name=None, password=None, timeout=None):
        interfaces.GeneratorBase.__init__(self)
        if not address or not user:
            raise ValueError("address='{}' and/or user='{}' cannot be empty or None.".format(address, user))
        self.address = address
        self.user = user
        self.name = name if name else utils.anon()
        self.password = password
        self.timeout = timeout
        self.env_loader = env_loader
#        self.ssh_session = None

#    def __del__(self):
#        if self.ssh_session:
#            self.ssh_session.close()

#    def __copy__(self):
#        # Ignore self.ssh_session (cannot / should not be copied).
#        cpy = Machine(self.address, self.user, self.name, self.password, self.timeout)
#        cpy.env_loader = self.env_loader
#        return cpy

#    def __deepcopy__(self):
#        # Ignore self.ssh_session (cannot / should not be copied).
#        dcpy = type(self)(copy.deepcopy(self.address, self.user, self.name, self.password, self.timeout))
#        dcpy.env_loader = copy.deepcopy(self.env_loader)
#        return dcpy

    def set_loader(self, script_path=None):
        self.env_loader = script_path

    def find(self, pkg, path_comp=None):
        # if not self.ssh_session:
        ssh_session = paramiko.SSHClient()
        ssh_session.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh_session.connect(hostname=self.address, username=self.user)
        # TODO: would be cooler if we can use our Python code remotely as well
        cmd = r"""
        if [ "$0" = "bash" ] || [ "$0" = "sh" ]; then
          . ~/.bashrc
        fi
        rospack find {:s}
        """.format(pkg.name)
        _, o, e = ssh_session.exec_command(cmd)

        o = [str(r).strip() for r in o.readlines()]
        e = [str(r).strip() for r in e.readlines()]
        if e or not o:
            raise RuntimeError(e)
        else:
            return o[0]

    def __eq__(self, other):
        return self.name == other.name

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.name)

    def __str__(self):
        return self.name

    def __repr__(self):
        pstr = ":{}".format(self.password) if self.password else str()
        return '{} -> {}{}@{}'.format(self.name, self.user, pstr, self.address)

    def generate(self, root, machines):
        elem = lxml.etree.Element('machine')
        root.insert(0, elem)  # insert at the top to make them usable in subsequent nodes
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        interfaces.GeneratorBase.to_attr(elem, 'address', self.address, str)
        interfaces.GeneratorBase.to_attr(elem, 'user', self.user, str)
        interfaces.GeneratorBase.to_attr(elem, 'password', self.password, str)
        interfaces.GeneratorBase.to_attr(elem, 'env-loader', self.env_loader, str)
        interfaces.GeneratorBase.to_attr(elem, 'timeout', self.timeout, float)


class MachinePool:  # TODO?
    def __init__(self):
        pass
