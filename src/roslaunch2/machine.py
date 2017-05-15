import lxml.etree
import getpass
import Pyro4
import ipaddress
import socket

import interfaces
import utils
import package
import remote


class Machine(interfaces.GeneratorBase):
    __generated_env_loaders = []

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

    def __resolve_setup(self):
        addr = self.address
        local = (addr == 'localhost' or addr == '127.0.0.1') and self.user == getpass.getuser()
        # Always use IP addresses as PYRONAMEs:
        try:
            ipaddress.ip_address(addr)
        except ValueError:
            addr = socket.gethostbyname(addr)
        return addr, local

    def resolve(self, what):
        addr, local = self.__resolve_setup()
        return what.resolve(addr, self.user, local)

    def remote(self, object_name=None):
        if not object_name:
            object_name = 'roslaunch2.remote.API'  # default to the roslaunch2 API

        def get_class(class_name):
            parts = class_name.split('.')
            module_name = ".".join(parts[:-1])
            m = __import__(module_name)
            for comp in parts[1:]:
                m = getattr(m, comp)
            return m
        addr, local = self.__resolve_setup()
        if local:
            return get_class(object_name)  # return a local instance
        return Pyro4.Proxy('PYRONAME:{:s}.{:s}.{:s}'.format(addr, self.user, object_name))

    def set_loader(self, script_path=None):
        self.env_loader = script_path

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

    def generate(self, root, machines, pkg):
        elem = lxml.etree.Element('machine')
        root.insert(0, elem)  # insert at the top to make them usable in subsequent nodes
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name, str)
        interfaces.GeneratorBase.to_attr(elem, 'address', self.address, str)
        interfaces.GeneratorBase.to_attr(elem, 'user', self.user, str)
        interfaces.GeneratorBase.to_attr(elem, 'password', self.password, str)
        # Automatically generate the env-loader script remotely:
        addr, local = self.__resolve_setup()
        if not self.env_loader and not local:
            pyro_addr = 'PYRONAME:{:s}.{:s}.roslaunch2.remote.Internals'.format(addr, self.user)
            Machine.__generated_env_loaders.append(pyro_addr)
            with Pyro4.Proxy(pyro_addr) as remote_object:
                self.env_loader = str(remote_object.get_env_loader())
        interfaces.GeneratorBase.to_attr(elem, 'env-loader', self.env_loader, str)
        interfaces.GeneratorBase.to_attr(elem, 'timeout', self.timeout, float)

    @staticmethod
    def resolve_if(what, machine_obj, pkg):
        if isinstance(what, remote.Resolvable):
            # Set package for resolving paths to provided pkg if not already set on construction:
            if isinstance(what, remote.Path):
                what.set_package(pkg)
            # Do the actual resolving
            if isinstance(machine_obj, Machine):  # resolve remotely
                return machine_obj.resolve(what)
            else:  # resolve on localhost
                return Localhost.resolve(what)
        else:
            return what  # unchanged (not resolvable)

    @staticmethod
    def cleanup():
        for obj_addr in Machine.__generated_env_loaders:
            with Pyro4.Proxy(obj_addr) as remote_object:
                remote_object.cleanup()
        Machine.__generated_env_loaders = []


Localhost = Machine('localhost', getpass.getuser())


class MachinePool:  # TODO?
    def __init__(self):
        pass
