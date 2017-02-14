import rospkg
import os

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
