#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Author: Adrian Böckenkamp
# License: BSD (https://opensource.org/licenses/BSD-3-Clause)
#    Date: 26/01/2018

import rospkg
import os
import sys
import Pyro4

import roslaunch2.logger


class Package:
    """
    Encapsulates a ROS package and its ability to find files in the package directory structure. A caching mechanism is
    used to speedup *find() commands.
    """
    __pkg_cache = {}
    __dir_cache = {}
    __find_cache = {}

    @staticmethod
    def invalidate_cache():
        Package.__pkg_cache = {}
        Package.__dir_cache = {}
        Package.__find_cache = {}

    @staticmethod
    def get_paths_to_file(start_dir, file_comp):
        """
        Searches for file_comp in $start_dir recursively (also using a cache for speedup).

        :param start_dir: root directory where to start the search
        :param file_comp: file path component (like some/dir/myfile.xml) name to search for
        :return: Set of files found (with their full path)
        """
        file_name = os.path.basename(file_comp)
        dir_comp = os.path.dirname(file_comp)
        result = []
        if start_dir in Package.__dir_cache:  # use cached file listing of $start_dir
            for root, file_set in Package.__dir_cache[start_dir]:
                for a_file in file_set:
                    if a_file == file_name and root.endswith(dir_comp):
                        result.append(os.path.join(root, a_file))
        else:  # crawl the file system at $start_dir (and cache for future requests)
            cache_entry = []
            for root, _, file_set in os.walk(start_dir):
                cache_entry.append((root, file_set))
                for a_file in file_set:
                    if a_file == file_name and root.endswith(dir_comp):
                        result.append(os.path.join(root, a_file))
            Package.__dir_cache[start_dir] = cache_entry
        return result

    @staticmethod
    def __get_pkg_path_cached(name):
        if name not in Package.__pkg_cache:
            Package.__pkg_cache[name] = rospkg.RosPack().get_path(name)  # may throws rospkg.ResourceNotFound
        return Package.__pkg_cache[name]

    def __init__(self, name=None):
        self.name = name
        self.path = Package.__get_pkg_path_cached(name)

    @Pyro4.expose
    def get_name(self):
        return self.name

    @Pyro4.expose
    def set_name(self, name):
        self.name = name
        self.path = Package.__get_pkg_path_cached(name)

    name = property(get_name, set_name)

    @Pyro4.expose
    def get_path(self):
        return self.path

    def _set_path(self, pkg_path):  # not exposed to Pyro!
        if self.name:
            self.path = pkg_path
        else:
            self.path = None

    path = property(get_path, _set_path)

    def __str__(self):
        return self.name

    @staticmethod
    def valid(pkg):
        """
        Tests whether pkg is a valid ROS package on the current system.

        :param pkg: Name of ROS package (type: str) or a valid package.Package object)
        :return: Path to pkg if valid or None if not found
        """
        try:
            if type(pkg) is str:
                name = pkg
            elif isinstance(pkg, Package):
                name = pkg.name
            else:
                raise ValueError('Cannot process type {}'.format(str(type(pkg))))
            return Package.__get_pkg_path_cached(name)
        except rospkg.ResourceNotFound:
            return None

    def has_node(self, node_name, warn=True):
        """
        Tests if a ROS node actually exists.

        This method checks whether a ROS node named $node_name exists in the current ROS package.

        :param node_name: name of ROS node to test
        :param warn: True if a warning about the missing node should be emitted
        :return: True if node exists, False otherwise
        """
        pkg = os.path.join(self.path, '../..')
        # Just consider files that are executable:
        if [f for f in Package.get_paths_to_file(pkg, node_name) if os.access(f, os.X_OK)]:
            # if len(res) > 1:
            #     log.warning("Found {} executable files named {}, assuming existence."
            #                 .format(len(res), node_name, res[0]))
            return True
        else:
            if warn:
                roslaunch2.logging.warning("Node '{}' in package '{}' not found.".format(node_name, self.name))
            return False

    @staticmethod
    def include(pkg_name, path_comp, **kwargs):
        """
        Like use() but static for convenience.

        :param pkg_name: Name of ROS package to be used for search of path_comp
        :param path_comp: (partial) path or file name to launch module (if it does not end with .pyl, this is added
               automatically)
        :param kwargs: optional arguments to be passed to the main() function of the launch module
        :return: GeneratorBase object as returned by the main() function
        """
        assert type(pkg_name) is str
        return Package(pkg_name).use(path_comp, **kwargs)

    def use(self, path_comp, **kwargs):
        """
        Imports (aka uses) the content of a launch module located in the current package (self).

        :param path_comp: (partial) path or file name to launch module (if it does not end with .pyl, this is added
               automatically)
        :param kwargs: optional arguments to be passed to the main() function of the launch module
        :return: GeneratorBase object as returned by the main() function
        """
        if not os.path.splitext(path_comp)[1]:
            path_comp += '.pyl'
        mod_path = self.find(path_comp, True)
        if not mod_path:
            raise ValueError("Launch module '{:s}' in package '{:s}' not found.".format(path_comp, self.name))
        m = Package.import_launch_module(mod_path)
        return m.main(**kwargs)

    @staticmethod
    def import_launch_module(full_module_path):
        if sys.version_info < (2, 4):  # Python < 2.4 is not supported
            raise RuntimeError('Must use Python version >= 2.4!')
        if not os.path.isfile(full_module_path):
            raise ValueError("Launch module '{:s}' not found.".format(full_module_path))
        module_name = os.path.splitext(full_module_path)[0]
        # Hot-patch PYTHONPATH to find . imports:
        search_path = os.path.dirname(os.path.abspath(module_name))
        if search_path not in sys.path:
            sys.path.append(search_path)
        if sys.version_info < (3, 3):  # Python 2.x and 3.x where x < 3
            import imp
            return imp.load_source(module_name, full_module_path)
        elif sys.version_info < (3, 4):  # Python 3.3 and 3.4
            import importlib.machinery
            return importlib.machinery.SourceFileLoader(module_name, full_module_path).load_module()
        elif sys.version_info >= (3, 5):  # Python 3.5+
            import importlib.util
            spec = importlib.util.spec_from_file_location(module_name, full_module_path)
            m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m)
            return m

    @Pyro4.expose
    def find(self, path_comp, silent=False):
        """
        Searches for a file or directory in the current package (self).

        :param path_comp: (partial) path or file name
        :param silent: if True return None when nothing is found, otherwise an IOError is raised in
                       case of failure
        :return: first found file (full path) or None if silent==True and nothing found
        """
        key = ''.join([self.name, path_comp])
        if key in Package.__find_cache:
            return Package.__find_cache[key]
        if not path_comp:
            return self.path
        dir_path = os.path.join(self.path, path_comp if not path_comp.startswith(os.path.sep) else path_comp[1:])
        if os.path.isdir(dir_path):
            Package.__find_cache[key] = dir_path
            return dir_path
        f = Package.get_paths_to_file(self.path, path_comp)
        if len(f) > 1:
            print("Found {} files, unique selection impossible (using first).".format(', '.join(f)))
        if not f:
            if not silent:
                raise IOError("No files like '{}' found in '{}'.".format(path_comp, self.name))
            else:
                return None
        Package.__find_cache[key] = f[0]
        return f[0]

    @Pyro4.expose
    def selective_find(self, path_comp_options, path_comp_prefix='', silent=False):
        """
        Searches for a set of files or directories in the current package (self). Tries to find any
        path from the path_comp_options list starting at the first element. Once a path is found
        the search for the remaining paths is canceled and the found path is returned.

        :param path_comp_options: list of (partial) path or file names
        :param path_comp_prefix: prefix to each element of path_comp_options
        :param silent: if True return None when nothing is found, otherwise an IOError is raised in
                       case of failure
        :return: first found file (full path) or None if silent==True and nothing found
        """
        for path_comp in path_comp_options:
            path = self.find(path_comp=os.path.join(path_comp_prefix, path_comp), silent=True)
            if path is not None:
                return path
        # Nothing found
        if not silent:
            raise IOError("None of the queried files found in '{}'.".format(self.name))
        else:
            return None
