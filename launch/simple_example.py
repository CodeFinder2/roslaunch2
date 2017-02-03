# roslaunch2 modules:
from roslaunch2.launch import Launch
from roslaunch2.node import Node, Output
from roslaunch2.node import
from roslaunch2.package import Package


def main():
    root = Launch()
    pkg = Package('rostopic')

    # Define a ROS node to launch (without namespace):
    root.add(Node(pkg, node='rostopic', name='rostopic', output=Output.Screen, args="list"))
    return root
