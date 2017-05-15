import os
import Pyro4
import utils
import tempfile

import package

__all__ = ["API", "Resolvable", "Path", "Variable"]


class Internals:
    __created_temp_files = []

    def __init__(self):
        pass

    @staticmethod
    @Pyro4.expose
    def get_env_loader():
        shell_name = os.environ['SHELL'].split(os.sep)[-1]
        if not shell_name:
            raise ValueError('Cannot determine / parse default shell.')
        # Create a temporary env-loader script which sets all environment variables that are currently set:
        ftmp = tempfile.NamedTemporaryFile(mode='w', suffix='-roslaunch2-temp-env.sh', delete=False)
        # Remember the file path for proper cleanups when the launch file exits:
        Internals.__created_temp_files.append(ftmp.name)
        content = '#!/usr/bin/env {:s}\n\n'.format(shell_name)
        content += '\n'.join(['export {:s}="{:s}"'.format(k, os.environ[k]) for k in os.environ])
        content += '\n\nexec "$@"\n\n'
        ftmp.write(content)
        ftmp.close()
        os.chmod(ftmp.name, 0o744)  # make it executable (set X bit)
        return ftmp.name

    @staticmethod
    @Pyro4.expose
    def cleanup():
        for f in Internals.__created_temp_files:
            utils.silent_remove(f)
            Internals.__created_temp_files = []


class API:
    def __init__(self):
        pass

    @staticmethod
    @Pyro4.expose
    def get_load_avg():
        return os.getloadavg()

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
    def __init__(self, data):
        self.data = data

    def resolve(self, address, user, local_only):
        raise NotImplementedError('resolve() not implemented in "{}" yet.'.format(self.__class__.__name__))


class Path(Resolvable):
    def __init__(self, path, pkg=None):
        Resolvable.__init__(self, path)
        self.pkg = package.Package(pkg) if type(pkg) is str else pkg

    def set_package(self, pkg):
        if not self.pkg:  # retain pkg initialized in ctor if already set (no overwrites)
            self.pkg = package.Package(pkg) if type(pkg) is str else pkg

    def resolve(self, address, user, local_only):
        if local_only:
            return self.pkg.find(self.data)
        remote_object = Pyro4.Proxy('PYRONAME:{:s}.{:s}.roslaunch2.package.Package'.format(address, user))
        remote_object.set_name(str(self.pkg))  # just the package name
        return str(remote_object.find(self.data))  # convert from unicode to str


class Variable(Resolvable):
    def __init__(self, name):
        Resolvable.__init__(self, name)

    def resolve(self, address, user, local_only):
        if local_only:
            return API.env(self.data)
        remote_object = Pyro4.Proxy('PYRONAME:{:s}.{:s}.roslaunch2.remote.API'.format(address, user))
        return str(remote_object.env(self.data))