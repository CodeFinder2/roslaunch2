import rospkg
import os


def get_paths_to_file(start_dir, file_comp):
    """
    Searches for $file_name in $start_dir recursively.

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
        self.path = rospkg.RosPack().get_path(name)

    def __str__(self):
        return self.name

    def find(self, path_comp):
        if not path_comp:
            return self.path
        dir_path = os.path.join(self.path, path_comp if not path_comp.startswith(os.path.sep) else path_comp[1:])
        if os.path.isdir(dir_path):
            return dir_path
        f = get_paths_to_file(self.path, path_comp)
        if len(f) > 1:
            print("Found {} files, unique selection impossible (using first).".format(', '.join(f)))
        if not f:
            raise RuntimeError("No files like '{}' found in '{}'.".format(path_comp, self.name))
        return f[0]
