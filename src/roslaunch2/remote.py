#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 26/01/2018

import os
import Pyro4
import utils
import tempfile

import package

__all__ = ["API", "Resolvable", "Path", "Variable"]


class Internals:
    """
    Contains various internal function used by the internals of roslaunch2. These functions should not be used from
    outside.
    """
    __created_temp_files = []

    def __init__(self):
        pass

    @staticmethod
    @Pyro4.expose
    def get_env_loader(env_vars):
        """
        Generate an env-loader script on the current machine given the current environment of the started
        roslaunch2_server.

        :param env_vars: Manually added environment variables that need to be available remotely
        :return: Path to generated env. script
        """
        shell_name = os.environ['SHELL'].split(os.sep)[-1]
        if not shell_name:
            raise ValueError('Cannot determine / parse default shell.')
        # Create a temporary env-loader script which sets all environment variables that are currently set:
        ftmp = tempfile.NamedTemporaryFile(mode='w', suffix='-roslaunch2-temp-env.sh', delete=False)
        # Remember the file path for proper cleanups when the launch file exits:
        Internals.__created_temp_files.append(ftmp.name)
        content = '#!/usr/bin/env {:s}\n\n'.format(shell_name)
        content += '\n'.join(['export {:s}="{:s}"'.format(k, os.environ[k]) for k in os.environ])
        content += '\n\n# Manually added environment variables (using roslaunch2.machine.Machine.set_env_var()):\n'
        content += '\n'.join(['export {:s}="{:s}"'.format(k, env_vars[k]) for k in env_vars])
        content += '\n\nexec "$@"\n\n'
        ftmp.write(content)
        ftmp.close()
        os.chmod(ftmp.name, 0o744)  # make it executable (set X bit)
        return ftmp.name

    @staticmethod
    @Pyro4.expose
    def cleanup():
        """
        Deletes all remotely created env-loader scripts.

        :return: None
        """
        for f in Internals.__created_temp_files:
            utils.silent_remove(f)
            Internals.__created_temp_files = []


class API:
    """
    Provides an API that can be used locally and remotely (via the PyRO backend) on other robots / machines. For
    example, it can be used to query the value of a remote environment variable or the number of CPU cores.
    """
    def __init__(self):
        pass

    @staticmethod
    @Pyro4.expose
    def load_avg():
        return os.getloadavg()

    @staticmethod
    @Pyro4.expose
    def memory_stats():
        """
        Determines the physical system-wide total (index 0) and available (index 1) memory (RAM) in bytes.

        :return: tuple (total, available) memory in bytes
        """
        import psutil
        return psutil.virtual_memory()[0:2]  # total and available memory in bytes

    @staticmethod
    @Pyro4.expose
    def env(name, optional=True):
        if str(name) not in os.environ and optional:
            return None
        return os.environ[str(name)]

    @staticmethod
    @Pyro4.expose
    def cpu_count():
        try:
            import multiprocessing
            return multiprocessing.cpu_count()
        except (ImportError, NotImplementedError):
            pass


class Resolvable(object):
    """
    Represents the interface for data that is just given as a "shell" or "sleeve" and still needs to be filled with the
    correct information of the target (remote) system. The latter process is denoted as resolving the data.
    """
    def __init__(self, data):
        self.data = data

    def resolve(self, address, user, local_only):
        raise NotImplementedError('resolve() not implemented in "{}" yet.'.format(self.__class__.__name__))


class Path(Resolvable):
    """
    Represents a path of a local or remote system. For example, if a node *may* be executed on another machine
    (depending on some condition) and that machine may be selected "dynamically" on some other condition, a path must be
    resolved if the final machine is known. This process is encapsulated in this class.
    """
    def __init__(self, path, pkg=None):
        Resolvable.__init__(self, path)
        self.pkg = package.Package(pkg) if type(pkg) is str else pkg

    def set_package(self, pkg):
        if not self.pkg:  # retain pkg initialized in ctor if already set (no overwrites)
            self.pkg = package.Package(pkg) if type(pkg) is str else pkg

    def resolve(self, address, user, local_only):
        if local_only:
            return self.pkg.find(self.data)
        with Pyro4.Proxy('PYRONAME:{:s}.{:s}.roslaunch2.package.Package'.format(address, user)) as remote_object:
            remote_object.set_name(str(self.pkg))  # just the package name
            return str(remote_object.find(self.data))  # convert from unicode to str


class Variable(Resolvable):
    """
    Represents an environment variable that is resolved on the localhost or remotely, depending on where the information
    is needed.
    """
    def __init__(self, name):
        Resolvable.__init__(self, name)

    def resolve(self, address, user, local_only):
        if local_only:
            return API.env(self.data)
        with Pyro4.Proxy('PYRONAME:{:s}.{:s}.roslaunch2.remote.API'.format(address, user)) as remote_object:
            return str(remote_object.env(self.data))
