#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 13/03/2018

import lxml.etree
import getpass
import Pyro4
import ipaddress
import socket
import enum

import interfaces
import utils
import remote


class Machine(interfaces.GeneratorBase):
    """
    For defining (remote) machines to launch on, equals the <machine> tag.
    """
    __generated_env_loaders = []

    def __init__(self, address, user, env_loader=None, name=None, password=None, timeout=None):
        """
        Initializes the Machine object.

        :param address: IP address or hostname of machine to be used for remote launching
        :param user: User name of remote machine
        :param env_loader: Optional env-loader script path on the local machine
        :param name: Unique name of machine object, can bet set to None to generate it automatically
        :param password: ssh password for connecting (it is advised to use public-key auth.)
        :param timeout: Optional timeout for connecting
        """
        interfaces.GeneratorBase.__init__(self)
        if not address or not user:
            raise ValueError("address='{}' and/or user='{}' cannot be empty or None.".format(address, user))
        self.address = address
        self.user = user
        self.__name = name
        self.password = password
        self.timeout = timeout
        self.env_loader = env_loader
        self.env_vars = {}

    def name(self):
        """
        Return the set machine name (self.__name) if set or generate name as hash of all relevant members.

        :return: Machine name as string
        """
        if self.__name is None:
            # Don't generate random machine name (utils.anon()) but use hash of all relevant members.
            return str(hash(tuple([self.address, self.user, self.env_loader] + list(frozenset(self.env_vars)))))
        return self.__name

    def __resolve_setup(self):
        """
        Resolve hostname if necessary and test if we should run on the local machine.

        :return: (IP addr, local_flag) tuple
        """
        addr = self.address
        # Always use IP addresses as PYRONAMEs:
        try:
            ipaddress.ip_address(addr)
        except ValueError:
            addr = socket.gethostbyname(addr)
        # Now, 'addr' must be an IP address.
        local = (addr == '127.0.0.1' and self.user == getpass.getuser())
        return addr, local

    def resolve(self, what):
        """
        Resolves the given object on the machine (convenience function).

        :param what: Object adhering to the remote.Resolvable interface
        :return: Resolved data
        """
        addr, local = self.__resolve_setup()
        return what.resolve(addr, self.user, local)

    def remote(self, object_name=None):
        """
        Allows one to remote execute a method on the given machine. For instance, m.remote().cpu_count() retrieves the
        number of CPUs on the machine "m".

        :param object_name: Pyro-exposed fully-qualified class instance name; can be empty to default to the roslaunch2
               API (roslaunch2.remote.API)
        :return: Object instance that allows to run methods locally or remotely depending on self
        """
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
        """
        Sets the env-loader script.

        :param script_path: Script path to the env-loader (must be valid on the remote host)
        """
        self.env_loader = script_path

    def __eq__(self, other):
        return self.name() == other.name()

    def __ne__(self, other):
        return not self.__eq__(other)

    def __hash__(self):
        return hash(self.name())

    def __str__(self):
        return self.name()

    def __repr__(self):
        pstr = ":{}".format(self.password) if self.password else str()
        return '{} -> {}{}@{}'.format(self.name(), self.user, pstr, self.address)

    def set_env_var(self, name, value):
        """
        Defines an environment variable on this machine iff the env-loader is generated automatically
        and the machine does not equal Localhost.

        :param name: Name of variable (should be unique)
        :param value: value to be assigned to the variable (converted to str)
        """
        self.env_vars[name] = str(value)

    def generate(self, root, machines, pkg):
        """
        Appends the underlying roslaunch XML code to the given root object.

        :param root: XML root element object
        :param machines: list of machines currently known in the launch module (may still contain duplicates)
        :param pkg: Package object, if none (else None); this is used / required on lower levels of the generation (see,
               e. g., ServerParameter.generate())
        """
        elem = lxml.etree.Element('machine')
        root.insert(0, elem)  # insert at the top to make them usable in subsequent nodes
        interfaces.GeneratorBase.to_attr(elem, 'name', self.name(), str)
        interfaces.GeneratorBase.to_attr(elem, 'address', self.address, str)
        interfaces.GeneratorBase.to_attr(elem, 'user', self.user, str)
        interfaces.GeneratorBase.to_attr(elem, 'password', self.password, str)
        # Automatically generate the env-loader script remotely:
        addr, local = self.__resolve_setup()
        if not self.env_loader and not local:
            pyro_addr = 'PYRONAME:{:s}.{:s}.roslaunch2.remote.Internals'.format(addr, self.user)
            Machine.__generated_env_loaders.append(pyro_addr)
            with Pyro4.Proxy(pyro_addr) as remote_object:
                self.env_loader = str(remote_object.get_env_loader(self.env_vars))
            # TODO: always generate env-loader (also on localhost to allow set_env_var() to work?)
        interfaces.GeneratorBase.to_attr(elem, 'env-loader', self.env_loader, str)
        interfaces.GeneratorBase.to_attr(elem, 'timeout', self.timeout, float)

    @staticmethod
    def resolve_if(what, machine_obj, pkg):
        """
        Tries to resolve "what" on the given "machine_obj", possibly relating to "pkg"

        :param what: Data sleeve to be resolved (should adhere to remote.Resolvable)
        :param machine_obj: Object of machine.Machine to be used for resolving
        :param pkg: ROS package instance
        :return: Resolved data or "what" (unchanged) if not resolvable
        """
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
        """
        Cleans up all env-loader script files generated remotely during the launch.
        """
        for obj_addr in Machine.__generated_env_loaders:
            with Pyro4.Proxy(obj_addr) as remote_object:
                remote_object.cleanup()
        Machine.__generated_env_loaders = []


Localhost = Machine('localhost', getpass.getuser())


class MachinePool(list):
    """
    Represents a set of machines to choose from when a new ROS node should be started based on a configurable criterion.
    """
    class Strategy(enum.IntEnum):
        LeastLoadAverage = 1
        LeastMemoryUsage = 2

        def __str__(self):
            if self.value == MachinePool.Strategy.LeastLoadAverage:
                return '\"LeastLoadAverage\"'
            elif self.value == MachinePool.Strategy.LeastMemoryUsage:
                return '\"LeastMemoryUsage\"'
            else:
                raise ValueError('Unknown strategy in machine pool!')

    def __init__(self, select_strategy=Strategy.LeastLoadAverage, *args):
        """
        Initializes the MachinePool.

        :param select_strategy: Strategy flag to select() a new machine
        :param args: Arguments to be passed to the underlying list constructor (e. g., Machine objects)
        """
        list.__init__(self, *args)
        self.strategy = select_strategy

    def by_address(self, address):
        """
        Returns the machine with the given address.

        :param address: User name to query
        :return: machine.Machine object or None if nothing was found
        """
        return [m for m in self if m.address == address]

    def by_user(self, user):
        """
        Returns the machine with the given user name.

        :param user: User name to query
        :return: machine.Machine object or None if nothing was found
        """
        return [m for m in self if m.user == user]

    def select(self):
        """
        Selects a machine for starting a node based on the current strategy. Note that this just considers the load on
        the machines in the pool when none of the nodes has actually been started (roslaunch XML generation step).

        :return: current optimal Machine object selected for execution
        """
        if not self:
            raise RuntimeError('Your MachinePool is empty! Cannot select() a machine for launching.')
        if self.strategy == MachinePool.Strategy.LeastLoadAverage:
            # Use 5min load average (divided by CPU count) value to determine the least used machine:
            fitness = [m.remote().load_avg()[1] / float(m.remote().cpu_count()) for m in self]
        elif self.strategy == MachinePool.Strategy.LeastMemoryUsage:
            # Use currently used memory to determine the most capable machine:
            fitness = [total - free for total, free in [m.remote().memory_stats() for m in self]]
        else:
            raise ValueError('Strategy {:s} is not implemented.'.format(str(self.strategy)))
        return self[fitness.index(min(fitness))]

    def __iadd__(self, other):
        """
        Behaves like list.append().

        :param other: Object to be added to this MachinePool (a Python list)
        :return: a reference to itself (self)
        """
        assert isinstance(other, Machine)
        self.append(other)
        return self
