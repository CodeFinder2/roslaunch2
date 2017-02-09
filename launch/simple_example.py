# roslaunch2 modules:
from roslaunch2 import *


def main():
    root = Launch()
    pkg = Package('rostopic')

    # Define a ROS node to launch (without namespace):
    root.add(Node(pkg, node='rostopic', name='rostopic', output=Output.Screen,
             args="pub /foo/bar std_msgs/String \"data: ''\""))
    return root
