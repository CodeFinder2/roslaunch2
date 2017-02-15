import rospkg
import os
import sys

import roslaunch2.logging


def get_paths_to_file(start_dir, file_comp):
    """
    Searches for file_comp in $start_dir recursively.

    :param start_dir: root directory where to start the search
    :param file_comp: file path component (like some/dir/myfile.xml) name to search for
    :return: Set of files found (with their full path)
    """
    file_name = os.path.basename(file_comp)
    dir_comp = os.path.dirname(file_comp)
    result = []
    for root, dir_set, file_set in os.walk(start_dir):
        for a_file in file_set:
            if a_file == file_name and root.endswith(dir_comp):
                result.append(os.path.join(root, a_file))
    return result


class Package:
    def __init__(self, name):
        self.name = name
        self.path = rospkg.RosPack().get_path(name)  # may throws rospkg.ResourceNotFound

    def __str__(self):
        return self.name

    @staticmethod
    def valid(pkg):
        try:
            if type(pkg) is str:
                name = pkg
            elif isinstance(pkg, Package):
                name = pkg.name
            else:
                raise ValueError('Cannot process type {}'.format(str(type(pkg))))
            pkg_path = rospkg.RosPack().get_path(name)
            return pkg_path
        except rospkg.ResourceNotFound:
            return None

    def has_node(self, node_name, warn=True):
        """
        Tests if a ROS node actually exists.

        This method checks whether a ROS node named $node_name exists in a ROS package named $package_name.

        :param node_name: name of ROS node to test
        :param warn: True if a warning about the missing node should be emitted
        :return: True if node exists, False otherwise
        """
        pkg = os.path.join(self.path, '../..')
        # Just consider files that are executable:
        if [f for f in get_paths_to_file(pkg, node_name) if os.access(f, os.X_OK)]:
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

        :param pkg_name:
        :param path_comp:
        :param kwargs:
        :return:
        """
        assert type(pkg_name) is str
        return Package(pkg_name).use(path_comp, **kwargs)

    def use(self, path_comp, **kwargs):
        mod_path = self.find(path_comp, True)
        if not mod_path:
            raise ValueError("Launch module '{:s}' in package '{:s}' not found.".format(path_comp, self.name))
        m = Package.import_launch_module(mod_path)
        return m.main(**kwargs)

    @staticmethod
    def import_launch_module(full_module_path):
        if sys.version_info < (2, 4):  # Python < 2.4 is not supported
            raise RuntimeError('Must use Python version >= 2.4!')
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

    def find(self, path_comp, silent=False):
        if not path_comp:
            return self.path
        dir_path = os.path.join(self.path, path_comp if not path_comp.startswith(os.path.sep) else path_comp[1:])
        if os.path.isdir(dir_path):
            return dir_path
        f = get_paths_to_file(self.path, path_comp)
        if len(f) > 1:
            print("Found {} files, unique selection impossible (using first).".format(', '.join(f)))
        if not f:
            if not silent:
                raise IOError("No files like '{}' found in '{}'.".format(path_comp, self.name))
            else:
                return None
        return f[0]
